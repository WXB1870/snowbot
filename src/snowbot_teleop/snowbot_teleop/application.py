"""PICO coordinator over shared position operations or legacy demo actuators.

Own operation flow and compose local mode/session capabilities. All motion and
actuator handoffs wait for fresh stop feedback; stale completions cannot rearm.
"""

import json
import math
import time
from snowbot_hardware.parameters import parameters_for
from copy import copy
from types import SimpleNamespace
from bindu_runtime.session_operation import SessionOperation
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.action import GoalResponse
from rclpy.task import Future
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from bindu_contracts.teleoperation import VRFrame, load_tool_config
from snowbot_teleop.control import (
    PicoControl,
    PicoFeedback,
    PICO_MAPPING,
    temporary_capture_refusal,
)
from . import preparation
from .status import operator_view
from bindu_kinematics.bimanual_worker import BimanualWorker
from bindu_interfaces.msg import VRInput, TeleopModeState
from bindu_interfaces.srv import Lease, ControlExecution
from bindu_interfaces.action import TeleopSession
from bindu_runtime.common import seconds, stamp, joints_at_rest
from bindu_runtime.stop_evidence import stop_feedback_matches
from bindu_runtime.execution_client import wait_for


class Superseded(RuntimeError):
    pass


class PicoApplication:
    def __init__(self, node, execution):
        self.node = node
        self.execution = execution
        self.profile = node.profile
        self.runtime_mode = node.runtime_mode
        self.simulation = node.simulation
        self.run_id = node.run_id
        self.instance_id = node.instance_id
        self.now = node.now
        self.event = node.event
        self.matches_mode = node.matches_mode
        self.get_logger = node.get_logger
        node.declare_parameter(
            "recording_enabled", True, descriptor=ParameterDescriptor(read_only=True)
        )
        self.recording_enabled = node.get_parameter("recording_enabled").value
        share = Path(get_package_share_directory("snowbot_bringup"))
        node.declare_parameter(
            "pico_config",
            str(share / "config/examples/snowbot/pico_snowbot_sim.json"),
            descriptor=ParameterDescriptor(read_only=True),
        )
        node.declare_parameter(
            "tool_config",
            str(share / "config/examples/snowbot/bimanual_snowbot_shovel.json"),
            descriptor=ParameterDescriptor(read_only=True),
        )
        node.declare_parameter(
            "model_root",
            str(Path(get_package_share_directory("snowbot_description")) / "urdf"),
            descriptor=ParameterDescriptor(read_only=True),
        )
        node.declare_parameter(
            "device_backend",
            "kinematic",
            descriptor=ParameterDescriptor(read_only=True),
        )
        if "position_protocol" in node._parameter_overrides:
            raise ValueError(
                "SNOWBOT_POSITION_PROTOCOL_REMOVED: session_v3 is fixed; remove position_protocol"
            )
        node.declare_parameter(
            "gripper_config", "", descriptor=ParameterDescriptor(read_only=True)
        )
        node.declare_parameter(
            "base_config", "", descriptor=ParameterDescriptor(read_only=True)
        )
        node.declare_parameter(
            "command_enabled", False, descriptor=ParameterDescriptor(read_only=True)
        )
        from snowbot_hardware.parameters import (
            load_parameters,
            apply_tool_timing,
            load_mode_config,
        )

        self.runtime_parameters = load_parameters(
            node.declare_parameter(
                "parameters_config", "", descriptor=ParameterDescriptor(read_only=True)
            ).value
            or None
        )
        self.base = None
        if node.get_parameter("base_config").value:
            from bindu_runtime.teleop_base import TeleopBase

            pico = self.runtime_parameters["pico"]
            self.base = TeleopBase(
                node,
                node.get_parameter("base_config").value,
                node.get_parameter("command_enabled").value,
                pair_max_skew_s=(
                    pico["pair_max_skew_s"] if "remote_input_timeout_s" in pico else 0.0
                ),
            )
        node.declare_parameter(
            "execution_mode", "position", descriptor=ParameterDescriptor(read_only=True)
        )
        self.execution_mode = node.get_parameter("execution_mode").value
        self.cfg = load_mode_config(
            node.get_parameter("pico_config").value, self.runtime_mode
        )
        node.declare_parameter(
            "position_contract", False, descriptor=ParameterDescriptor(read_only=True)
        )
        self.position_contract = node.get_parameter("position_contract").value
        if self.cfg.get("schema_version") != (2 if self.position_contract else 1):
            raise ValueError("PICO_CONFIG_SCHEMA")
        if self.cfg.get("mapping") != PICO_MAPPING:
            raise ValueError("PICO_CONFIG_RUNTIME_MODE_MISMATCH")
        from bindu_runtime.teleop_deployment import execution_backend

        self.backend = execution_backend(
            self.runtime_mode,
            node.get_parameter("device_backend").value,
            self.execution_mode,
        )
        if not self.position_contract and self.backend != "kinematic":
            raise ValueError("PICO_ACTUATOR_BACKEND_UNAVAILABLE:" + self.backend)
        self.tool_cfg = load_tool_config(node.get_parameter("tool_config").value)
        if self.position_contract:
            self.tool_cfg = apply_tool_timing(self.tool_cfg, self.runtime_parameters)
        self.automatic_close_grippers = self.cfg.get("automatic_close_grippers", False)
        if type(self.automatic_close_grippers) is not bool or (
            self.automatic_close_grippers and not self.position_contract
        ):
            raise ValueError("PICO_AUTO_CLOSE_CONFIG")
        policy = self.tool_cfg.get("measured_grasp_policy", "enforce")
        if policy not in ("enforce", "unheld_test"):
            raise ValueError("PICO_MEASURED_GRASP_POLICY")
        self.unheld_test = policy == "unheld_test"
        confirmation = (
            self.cfg.get("tool_grasp_confirmation")
            if self.position_contract
            else self.tool_cfg.get("tool_grasp_confirmation", "calibrated")
        )
        if confirmation not in ("calibrated", "operator"):
            raise ValueError("PICO_TOOL_GRASP_CONFIRMATION_CONFIG")
        self.operator_grasp = confirmation == "operator"
        self.operator_grasp_positions = {}
        if (
            self.operator_grasp
            and not self.unheld_test
            and (
                not self.position_contract
                or self.tool_cfg.get("grasp_reference_source")
                not in ("measured_rigid", "measured_axial")
            )
        ):
            raise ValueError("PICO_OPERATOR_GRASP_REQUIRES_MEASURED_REFERENCE")
        if (
            not self.tool_cfg.get("right_axial_grasp")
            or self.tool_cfg.get("collision_scope") != "configured_tool_geometry"
            or not self.position_contract
            and not self.tool_cfg.get("demo", {}).get("initial_positions_rad")
        ):
            raise ValueError("PICO_REQUIRES_AXIAL_SHOVEL_CONFIG")
        self.tool_cfg["urdf"] = str(
            Path(node.get_parameter("model_root").value) / self.tool_cfg["urdf"]
        )
        self.groups = tuple(self.tool_cfg["resource_groups"]) + (
            ()
            if self.position_contract
            else tuple(self.cfg["grippers"][s]["group"] for s in ("left", "right"))
        )
        if len(set(self.groups)) != len(self.groups) or any(
            g not in self.profile.groups for g in self.groups
        ):
            raise ValueError("PICO_RESOURCE_LAYOUT")
        self.names = tuple(n for g in self.groups for n in self.profile.groups[g])
        for side in () if self.position_contract else ("left", "right"):
            item = self.cfg["grippers"][side]
            if tuple(self.profile.groups[item["group"]]) != (item["joint"],):
                raise ValueError("PICO_GRIPPER_LAYOUT")
            targets = [item[k] for k in ("open", "closed", "virtual")]
            if (
                any(
                    type(q) not in (float, int)
                    or not math.isfinite(q)
                    or not self.profile.limits[item["joint"]][0]
                    <= q
                    <= self.profile.limits[item["joint"]][1]
                    for q in targets
                )
                or len(set(targets)) != 3
            ):
                raise ValueError("PICO_GRIPPER_TARGETS")
        initial_load = self.cfg.get("initial_load_state", "unknown")
        if initial_load not in ("unknown", "empty") or (
            initial_load != "unknown"
            and not (self.simulation and self.position_contract)
        ):
            raise ValueError("PICO_STARTUP_LOAD_MUST_BE_UNKNOWN")
        if not self.simulation and self.cfg.get("device_initial_positions_rad"):
            raise ValueError("REAL_REQUIRES_MEASURED_INITIAL_STATE")
        node.declare_parameter("simulation_demo", False)
        self.simulation_demo = node.get_parameter("simulation_demo").value
        if type(self.simulation_demo) is not bool or (
            self.simulation_demo
            and (
                not self.simulation
                or self.backend != "kinematic"
                or self.position_contract
            )
        ):
            raise ValueError("PICO_DEMO_REQUIRES_KINEMATIC_SIMULATION")
        if self.simulation_demo:
            from bindu_teleoperation.simulation_fixture import short_rod_config

            self.tool_cfg = short_rod_config(self.tool_cfg)
        self.prepare_positions = (
            self.cfg.get("prepare_positions_rad")
            if self.position_contract
            else (
                self.cfg.get(
                    "simulation_demo_prepare_positions_rad",
                    self.tool_cfg["demo"]["initial_positions_rad"],
                )
                if self.simulation_demo
                else self.tool_cfg["demo"]["initial_positions_rad"]
            )
        )
        self.primitive_library = None
        if self.position_contract and self.cfg.get("primitive_library"):
            from bindu_contracts.resources import resource_file
            from snowbot_teleop.replay import replay_graph, NAMES

            self.primitive_library = resource_file(self.cfg["primitive_library"])
            library, _, _ = replay_graph(self.primitive_library, 1.0, None, 0.0)
            self.prepare_positions = dict(
                zip(NAMES, library["nodes"][library["home_node"]])
            )
        prepare_names = [
            n for g in self.tool_cfg["resource_groups"] for n in self.profile.groups[g]
        ]
        if self.prepare_positions is not None and (
            not set(prepare_names) <= set(self.prepare_positions)
            or any(
                not math.isfinite(self.prepare_positions[n])
                or not self.profile.limits[n][0]
                <= self.prepare_positions[n]
                <= self.profile.limits[n][1]
                for n in prepare_names
            )
        ):
            raise ValueError("PICO_PREPARE_PRESET_INVALID")
        if not 0 < float(self.cfg["prepare_duration"]) <= 60:
            raise ValueError("PICO_PREPARE_DURATION_INVALID")
        self.prepare_position_tolerance = float(
            self.cfg.get("prepare_position_tolerance_rad", 0.002)
        )
        if (
            not math.isfinite(self.prepare_position_tolerance)
            or self.prepare_position_tolerance <= 0
        ):
            raise ValueError("PICO_PREPARE_TOLERANCE_INVALID")
        self.prepare_limit_max = float(
            self.cfg.get("prepare_limit_recovery_max_rad", 0.0)
        )
        if (
            not math.isfinite(self.prepare_limit_max)
            or not 0 <= self.prepare_limit_max <= self.prepare_position_tolerance
        ):
            raise ValueError("PICO_PREPARE_LIMIT_RECOVERY_CONFIG")
        if self.simulation:
            self.prepare_limit_max = 0.0
        self.control = PicoControl(
            initial_load=initial_load,
            simulation_demo=self.simulation_demo,
            max_age=(
                self.runtime_parameters["teleop"]["input_max_age_s"]
                if self.position_contract
                else float(self.tool_cfg["input_max_age"])
            ),
            guided_loading=self.position_contract,
            dual_modes=self.position_contract,
            direct_grippers=self.position_contract,
            initial_mode=self.cfg.get("initial_teleop_mode", "independent_arms"),
            pose_jump_translation_m=self.cfg.get("pose_jump_translation_m", 0.35),
            pose_jump_rotation_rad=self.cfg.get("pose_jump_rotation_rad", 1.2),
            settings=self.runtime_parameters["pico"],
        )
        validation_cfg = dict(
            self.tool_cfg,
            worker_timeout=self.runtime_parameters["pico"][
                "validation_worker_timeout_s"
            ],
        )
        recovery = self.cfg.get("prepare_collision_recovery")
        if recovery is not None:
            if not self.simulation or not self.position_contract:
                raise ValueError("PICO_PREPARE_RECOVERY_REQUIRES_SIMULATION_POSITION")
            from bindu_kinematics.tool_geometry import InitialSelfCollisionRecovery

            validation_cfg["prepare_collision_recovery"] = InitialSelfCollisionRecovery(
                recovery
            ).config
        validation_cfg["prepare_limit_recovery_max_rad"] = self.prepare_limit_max
        validation_cfg["prepare_reference_max_offset_rad"] = (
            self.prepare_position_tolerance
        )
        self.validator = BimanualWorker(
            validation_cfg, self.profile, preload_path_checks=True
        )
        self.validation_result = None
        self.prepare_recovery = None
        self.prepare_limit_reference = None
        self.prepare_limit_start = None
        self.frames = {}
        self.mode = None
        self.handoff_complete = False
        self.tool_goal = None
        self.tool_result = None
        self.tool_phase = "inactive"
        self.lease = None
        self.pending_intent = None
        self.running = None
        self.waiters = set()
        self.stop_request = None
        self.confirmed_stop = None
        self.release_until = 0.0
        self.recording = None
        self.record_future = None
        self.tool_request = None
        self.last_status = 0.0
        self.last_phase = None
        self.last_prepare_check = None
        self.last_context_at = self.control.context_at
        self.last_pair_at = None
        self.last_tick_at = None
        self.motion_limit = None
        self.motion_limit_stamp = 0.0
        self.position_ops = None
        if self.position_contract:
            from .position import PositionOperations

            self.position_ops = PositionOperations(
                self, node.get_parameter("gripper_config").value
            )
        self.independent_grippers = bool(
            self.position_ops
            and self.position_ops.config.get("independent_trigger_control", False)
        )
        self.control.independent_grippers = self.independent_grippers
        self.control.continuous_axial = (
            self.tool_cfg.get("grasp_reference_source") == "measured_axial"
        )
        self.leases = execution.leases
        self.submit = execution.submit
        self.stop_client = execution.control
        self.prepare_hold_client = (
            execution.held_reference
            if self.position_contract and not self.simulation
            else None
        )
        self.record_client = None
        self.tool = None
        self.selection = None
        self.last_mode_status = float("-inf")

    @property
    def state(self):
        return self.execution.state

    @property
    def executor(self):
        return self.node.executor

    def attach_tool(self, tool):
        from bindu_teleoperation.modes import ModeSelection

        self.tool = tool
        self.selection = ModeSelection(
            self.profile,
            "left",
            self.now(),
            input_age=tool.cfg["input_max_age"],
            feedback_age=tool.cfg["feedback_max_age"],
            grip_release_threshold=tool.cfg["grip_release_threshold"],
            pair_max_skew_s=tool.cfg["pair_max_skew_s"],
            left_orientation="right_axial_grasp" in tool.cfg,
            require_recording=self.recording_enabled,
            enabled_modes=(("independent_arms",) if self.position_contract else ())
            + ("bimanual_tool",),
        )

    def publish_mode(self):
        m = self.selection
        state = TeleopModeState(
            stamp=stamp(self.now()),
            changed_at=stamp(m.changed_at),
            profile_hash=self.profile.digest,
            runtime_mode=self.runtime_mode,
            generation=m.generation,
            mode=m.mode,
            side=m.side,
            phase=m.phase,
            code=m.code,
        )
        self.mode = self.tool.mode_state = state
        self.mode_output.publish(state)
        self.last_mode_status = self.now()

    def on_recording(self, msg):
        if (
            self.matches_mode(msg)
            and msg.run_id == self.run_id
            and 0 <= self.now() - seconds(msg.stamp) < 0.75
        ):
            self.recording = msg.capture_enabled

    def reading(self):
        s = self.state
        if s and (
            (self.lease is not None and s.instance_id != self.lease[1])
            or (
                self.tool_request is not None
                and (
                    s.instance_id != self.tool_request["instance_id"]
                    or s.lease_active
                    and s.lease_epoch != self.tool_request["epoch"]
                )
            )
        ):
            raise RuntimeError("PICO_EXECUTION_AUTHORITY_CHANGED")
        if s is not None and s.stop_failures:
            raise RuntimeError("PICO_STOP_UNCONFIRMED:" + s.code)
        if (
            s is not None
            and not 0
            <= self.now() - seconds(s.feedback_stamp)
            <= self.tool_cfg["feedback_max_age"]
        ):
            raise RuntimeError("PICO_FEEDBACK_STALE")
        if (
            s is None
            or s.profile_hash != self.profile.digest
            or s.device_backend != self.backend
            or s.simulated != self.simulation
            or len(s.joints.name) != len(s.joints.position)
            or len(set(s.joints.name)) != len(s.joints.name)
            or not all(math.isfinite(q) for q in s.joints.position)
        ):
            raise RuntimeError("PICO_FEEDBACK_UNAVAILABLE")
        measured = dict(zip(s.joints.name, s.joints.position))
        if not set(self.profile.limits) <= set(measured):
            raise RuntimeError("PICO_FEEDBACK_LAYOUT")
        return s, measured

    def feedback(self):
        try:
            s, q = self.reading()
            if self.position_ops and getattr(self, "independent_grippers", False):
                for side in ("left", "right"):
                    self.position_ops.reading(side)
            modes = (
                list(
                    self.position_ops.modes(
                        require_stationary=self.control.phase != "following"
                    )
                )
                if self.position_ops
                else []
            )
            for side in () if self.position_ops else ("left", "right"):
                cfg = self.cfg["grippers"][side]
                value = q[cfg["joint"]]
                modes.append(
                    next(
                        (
                            m
                            for m in ("closed", "open", "virtual")
                            if abs(value - cfg[m]) < 0.003
                        ),
                        "unknown",
                    )
                )
            stopped = (
                (self.handoff_complete or not s.lease_active)
                and s.state not in ("RUNNING", "STOPPING")
                and not s.reference.name
                and joints_at_rest(
                    s.joints,
                    self.names,
                    self.profile.feedback_tolerances.get("joint_velocity", 0.01),
                )
                and (not self.position_ops or self.position_ops.stopped())
            )
            if self.base and self.base.writable and self.base.cfg["require_feedback"]:
                clock = time.monotonic()
                if not self.base.core.healthy(self.now(), clock):
                    raise RuntimeError("BASE_FEEDBACK_UNAVAILABLE")
                stopped = stopped and self.base.core.stopped(self.now(), clock)
            # A new executor's idle display cannot erase an outstanding stop.
            if self.stop_request is not None:
                if (
                    self.stop_request["failed"]
                    or time.monotonic() >= self.stop_request["deadline"]
                ):
                    raise RuntimeError("PICO_STOP_UNCONFIRMED")
                stopped = False
            return PicoFeedback(seconds(s.feedback_stamp), stopped, tuple(modes))
        except RuntimeError as exc:
            return PicoFeedback(0.0, False, healthy=False, fault=str(exc))

    def on_context(self, req, reply):
        self.control.update_feedback(self.feedback(), self.now())
        try:
            expected = (
                (self.control.source + ":" + self.instance_id)
                if self.control.source
                else None
            )
            if req.source_id != expected:
                raise ValueError("PICO_COORDINATOR_SOURCE_CHANGED")
            self.control.context(
                self.control.source,
                req.expected_generation,
                req.load_state,
                req.external_support,
                self.now(),
            )
            reply.success, reply.code = (
                True,
                "OPERATOR_DECLARATION_ACCEPTED_NOT_SENSOR_PROOF",
            )
            self.event("PICO_CONTEXT", json.dumps(self.control.snapshot()))
        except ValueError as exc:
            reply.success, reply.code = False, str(exc)
        reply.generation = self.control.generation
        return reply

    def on_motion_event(self, msg):
        if (
            msg.run_id != self.run_id
            or msg.runtime_mode != self.runtime_mode
            or msg.component != "execution"
            and msg.task_id != "pico_" + self.instance_id
        ):
            return
        if msg.component == "execution":
            owned = (
                msg.task_id == "pico_" + self.instance_id
                or getattr(msg, "command_id", "").startswith(self.instance_id + "_")
            ) or (
                not msg.task_id
                and (
                    self.tool_request is not None
                    or self.lease is not None
                    or self.control.fault_active
                )
            )
            if not owned:
                return
            if msg.state == "STOP_CHECK":
                try:
                    detail = json.loads(msg.code)
                except (ValueError, TypeError):
                    return
                if not isinstance(detail, dict):
                    return
                self.stop_diagnostic = detail
                observed = detail.get("observed") or {}
                if (
                    isinstance(observed, dict)
                    and type(observed.get("session_reason")) is int
                    and observed["session_reason"] >= 3
                ):
                    code = "SNOWBOT_SESSION_STOPPED:" + str(observed["session_reason"])
                    if self.control.record_fault(code, seconds(msg.stamp), device=True):
                        self.control.lock(code, self.now())
            elif msg.state in (
                "STOPPING",
                "FAILED",
                "DEVICE_FEEDBACK_DETAIL",
                "DEVICE_COMMAND_DETAIL",
            ) and msg.code not in ("CANCELED", "OK", "SHUTDOWN"):
                if self.control.record_fault(msg.code, seconds(msg.stamp)):
                    self.control.lock(msg.code, self.now())
            return
        if msg.state in ("BIMANUAL_CAPTURED", "BIMANUAL_ENDED"):
            self.motion_limit = None
        elif msg.state == "BIMANUAL_QP_RESULT":
            try:
                value = json.loads(msg.code)
                self.motion_limit = value.get("geometry_step")
                self.motion_limit_stamp = seconds(msg.stamp)
            except (ValueError, TypeError):
                pass

    def on_frame(self, msg):
        if not self.matches_mode(msg) or msg.side not in ("left", "right"):
            return
        if self.control.superseded_remote_input(msg.source_id, msg.seq):
            return
        self.frames[msg.side] = msg
        if set(self.frames) != {"left", "right"}:
            return
        a, b = self.frames["left"], self.frames["right"]
        if a.source_id != b.source_id or a.seq != b.seq:
            return
        now = self.now()
        received_monotonic = time.monotonic()
        previous_monotonic = getattr(self, "last_pair_monotonic", None)
        monotonic_gap = (
            None
            if previous_monotonic is None
            else received_monotonic - previous_monotonic
        )
        warning_s = self.runtime_parameters["diagnostics"]["input_gap_warning_s"]
        if self.last_pair_at is not None and (
            now - self.last_pair_at > warning_s
            or monotonic_gap is not None
            and monotonic_gap > warning_s
        ):
            self.event(
                "PICO_INPUT_DELIVERY_GAP",
                json.dumps(
                    dict(
                        interval_s=now - self.last_pair_at,
                        interval_clock="ros_callback",
                        monotonic_interval_s=monotonic_gap,
                        input_age_s=now - seconds(a.stamp),
                        source_stamp_s=seconds(a.stamp),
                        source_id=a.source_id,
                        seq=a.seq,
                    )
                ),
            )
        self.last_pair_at = now
        self.last_pair_monotonic = received_monotonic
        pair = tuple(
            VRFrame(
                m.source_id,
                m.seq,
                seconds(m.stamp),
                m.side,
                tuple(m.pose),
                m.grip,
                m.trigger,
                m.init,
                m.stop,
                m.valid,
                m.clutch_seq,
                m.mapping,
                m.buttons,
                tuple(int(v) for v in m.button_edges),
                m.virtual_grip,
                tuple(m.stick),
                head_pose=tuple(getattr(m, "head_pose", ())),
                head_stamp=(
                    seconds(m.head_stamp) if getattr(m, "head_pose", ()) else 0.0
                ),
                head_age_s=getattr(m, "head_age_s", 0.0),
                head_source_id=getattr(m, "head_source_id", ""),
                head_seq=int(getattr(m, "head_seq", 0)),
            )
            for m in (a, b)
        )
        if self.control.expired_remote_pair(pair, now):
            return
        if self.base:
            self.base.ingest(pair, now)
        self.control.update_feedback(self.feedback(), self.now())
        previous = (self.control.source, self.control.seq)
        previous_frames = self.control.frames
        previous_reason = self.control.reason
        previous_jump = self.control.last_pose_jump
        self.control.ingest(*pair, self.now())
        if self.control.last_pose_jump is not previous_jump:
            self.event("PICO_POSE_JUMP", json.dumps(self.control.last_pose_jump))
        if self.control.last_prepare_check is not self.last_prepare_check:
            self.last_prepare_check = self.control.last_prepare_check
            self.event("PICO_PREPARE_CHECK", json.dumps(self.last_prepare_check))
        if (
            self.control.reason == "PICO_PAIR_INVALID"
            and previous_reason != self.control.reason
        ):
            self.event(
                "PICO_PAIR_DIAGNOSTIC",
                json.dumps(
                    dict(
                        now=self.now(),
                        frames=[
                            dict(
                                stamp=f.stamp,
                                seq=f.seq,
                                side=f.side,
                                source=f.source_id,
                                mapping=f.mapping,
                                valid=f.valid,
                                init=f.init,
                                grip=f.grip,
                                trigger=f.trigger,
                                buttons=f.buttons,
                                edges=f.button_edges,
                                clutch_seq=f.clutch_seq,
                            )
                            for f in pair
                        ],
                    )
                ),
            )
        if (
            self.control.reason == "PICO_TIME_REVERSED"
            and previous_reason != self.control.reason
        ):
            self.event(
                "PICO_TIME_DIAGNOSTIC",
                json.dumps(
                    dict(
                        now=self.now(),
                        source=a.source_id,
                        previous_seq=previous[1],
                        input_seq=a.seq,
                        frames=[
                            dict(
                                side=new.side,
                                previous_stamp=old.stamp,
                                input_stamp=new.stamp,
                                interval_s=new.stamp - old.stamp,
                            )
                            for old, new in zip(previous_frames, pair)
                        ],
                    )
                ),
            )
        if (
            self.control.source != a.source_id
            or self.control.seq != a.seq
            or previous == (a.source_id, a.seq)
        ):
            return
        allowed = (
            self.control.phase in ("engaging", "following")
            and self.now() >= self.release_until
        )
        publish_started = time.monotonic()
        for i, m in enumerate((a, b)):
            hand_allowed = allowed and (
                self.control.teleop_mode != "independent_arms"
                or self.control.squeeze_released[i]
            )
            projected = VRInput(
                stamp=m.stamp,
                source_id=m.source_id + ":" + self.instance_id,
                runtime_mode=m.runtime_mode,
                seq=m.seq,
                side=m.side,
                pose=m.pose,
                grip=m.grip if hand_allowed else 0.0,
                trigger=(
                    m.trigger
                    if getattr(self, "independent_grippers", False)
                    or not self.position_contract
                    else 0.0
                ),
                init=False,
                stop=self.control.phase == "locked" or not self.handoff_complete,
                valid=m.valid,
                clutch_seq=m.clutch_seq,
                mapping=PICO_MAPPING,
                buttons=0 if self.position_contract else m.buttons,
                button_edges=(
                    [0] * 6 + list(m.button_edges[6:8])
                    if getattr(self, "independent_grippers", False)
                    else [0] * 8 if self.position_contract else m.button_edges
                ),
                virtual_grip=self.control.virtual,
                stick=m.stick,
                head_pose=getattr(m, "head_pose", ()),
                head_stamp=getattr(m, "head_stamp", m.stamp),
                head_age_s=getattr(m, "head_age_s", 0.0),
                head_seq=int(getattr(m, "head_seq", 0)),
                head_source_id=(
                    m.head_source_id + ":" + self.instance_id
                    if getattr(m, "head_source_id", "") == m.source_id
                    else getattr(m, "head_source_id", "")
                ),
            )
            self.tool.on_frame(projected)
            if self.projected.get_subscription_count():
                self.projected.publish(projected)
            if self.independent_grippers:
                # Ordinary arm handoff must not erase independent trigger edges.
                # Keep the arm stream fenced and all global gripper stops intact.
                gripper_frame = copy(projected)
                gripper_frame.stop = (
                    m.stop
                    or self.control.phase in ("checking", "locked")
                    or self.control.fault_active
                    or bool(self.control.stop_failure or self.control.recovery_required)
                )
                self.gripper_projected.publish(gripper_frame)
        self.projected_publish_s = time.monotonic() - publish_started

    async def pause(self):
        future = Future(executor=self.executor)
        self.waiters.add(future)
        try:
            await future
        finally:
            self.waiters.discard(future)

    def current(self, intent):
        if not self.control.pending or self.control.pending.token != intent.token:
            raise Superseded("PICO_OPERATION_SUPERSEDED")
        self.control.tick(self.now())
        if not self.control.pending or self.control.pending.token != intent.token:
            raise Superseded("PICO_OPERATION_SUPERSEDED")

    async def wait(self, future, timeout=None):
        if timeout is None:
            timeout = parameters_for(self, "pico")["rpc_timeout_s"]
        return await wait_for(
            future, self.pause, timeout, "PICO_RPC_TIMEOUT", clock=time.monotonic
        )

    def begin_stop_wait(self, instance_id, epoch, lease_active):
        if self.stop_request is not None:
            self.stop_unconfirmed(self.stop_request, "PREVIOUS_REQUEST_UNCONFIRMED")
        self.confirmed_stop = None
        self.stop_diagnostic = None
        self.stop_request = dict(
            instance_id=instance_id,
            epoch=epoch,
            lease_active=lease_active,
            requested=self.now(),
            deadline=time.monotonic()
            + getattr(getattr(self, "profile", None), "stop_timeout", 5.0)
            + parameters_for(self, "pico")["stop_wait_margin_s"],
            failed=False,
        )
        self.handoff_complete = False
        return self.stop_request

    def stop_unconfirmed(self, request, reason):
        # A delayed coroutine must not fail/clear a newer request or goal.
        if request is self.stop_request and request["failed"]:
            raise RuntimeError(
                "PICO_STOP_UNCONFIRMED:"
                + request.get("failure_reason", "PREVIOUS_REQUEST_UNCONFIRMED")
            )
        self.control.record_fault(
            "PICO_STOP_UNCONFIRMED:" + reason,
            self.now(),
            kind="request" if reason == "REQUEST_NOT_CURRENT" else "stop",
        )
        if request is self.stop_request and not request["failed"]:
            request["failed"] = True
            request["failure_reason"] = reason
            s = self.state
            self.event(
                "PICO_STOP_UNCONFIRMED",
                json.dumps(
                    dict(
                        reason=reason,
                        requested_instance_id=request["instance_id"],
                        expected_epoch=request["epoch"],
                        expected_lease_active=request["lease_active"],
                        requested_at=request["requested"],
                        deadline_monotonic=request["deadline"],
                        observed_instance_id=s.instance_id if s else None,
                        observed_epoch=s.lease_epoch if s else None,
                        state=s.state if s else None,
                        code=s.code if s else None,
                    )
                ),
            )
        raise RuntimeError("PICO_STOP_UNCONFIRMED:" + reason)

    async def stopped(self, request, *, clear=True, lease=None):
        if request is not self.stop_request:
            self.stop_unconfirmed(request, "REQUEST_NOT_CURRENT")
        if request["failed"]:
            raise RuntimeError(
                "PICO_STOP_UNCONFIRMED:"
                + request.get("failure_reason", "PREVIOUS_REQUEST_UNCONFIRMED")
            )
        next_renewal = 0.0
        while time.monotonic() < request["deadline"]:
            if request is not self.stop_request:
                self.stop_unconfirmed(request, "REQUEST_NOT_CURRENT")
            if request["failed"]:
                raise RuntimeError(
                    "PICO_STOP_UNCONFIRMED:"
                    + request.get("failure_reason", "PREVIOUS_REQUEST_UNCONFIRMED")
                )
            if lease is not None and time.monotonic() >= next_renewal:
                reply = await self.wait(
                    self.leases.call_async(
                        Lease.Request(
                            operation="renew",
                            lease_id=lease.lease_id,
                            epoch=lease.epoch,
                        )
                    ),
                    min(0.7, max(0.0, request["deadline"] - time.monotonic())),
                )
                if not (
                    reply.ok
                    and reply.lease_id == lease.lease_id
                    and reply.epoch == lease.epoch
                    and reply.profile_hash == self.profile.digest
                ):
                    self.stop_unconfirmed(request, "LEASE_RENEW_REJECTED:" + reply.code)
                next_renewal = (
                    time.monotonic()
                    + parameters_for(self, "pico")["stop_renewal_period_s"]
                )
            s = self.state
            if s and (
                s.stop_failures
                or "EMERGENCY_STOP" in s.code
                or s.code == "STOP_FEEDBACK_TIMEOUT"
            ):
                self.stop_unconfirmed(request, "EXECUTION_STOP_FAILED")
            try:
                s, _ = self.reading()
            except RuntimeError:
                await self.pause()
                continue
            if stop_feedback_matches(
                s,
                instance_id=request["instance_id"],
                epoch=request["epoch"],
                lease_active=request["lease_active"],
                profile_hash=self.profile.digest,
                simulated=self.simulation,
                requested=request["requested"],
                now=self.now(),
                max_age=self.tool_cfg["feedback_max_age"],
                at_rest=lambda joints: joints_at_rest(
                    joints,
                    self.names,
                    self.profile.feedback_tolerances.get("joint_velocity", 0.01),
                ),
            ) and (not self.position_ops or self.position_ops.stopped()):
                if clear:
                    self.confirmed_stop = dict(request)
                    self.stop_request = None
                    if not request["lease_active"]:
                        self.handoff_complete = True
                return
            await self.pause()
        self.stop_unconfirmed(request, "DEADLINE_EXPIRED")

    async def cancel_tool(self):
        if self.stop_request is not None:
            return await self.stopped(self.stop_request)
        if self.lease is not None:
            await self.release_lease()
        goal, result = self.tool_goal, self.tool_result
        context = self.tool_request
        if context is None:
            s = self.state
            request = self.begin_stop_wait(
                s.instance_id if s and goal is None else "",
                s.lease_epoch if s else -1,
                False,
            )
        else:
            context["canceling"] = True
            request = self.begin_stop_wait(
                context["instance_id"], context["epoch"] + 1, False
            )
        try:
            if goal is not None:
                goal.cancel()
                if result is None:
                    self.stop_unconfirmed(request, "TOOL_RESULT_MISSING")
                ended = await self.wait(
                    result, min(7.0, max(0.0, request["deadline"] - time.monotonic()))
                )
                code = ended.code
                if "STOP_UNCONFIRMED" in code:
                    self.stop_unconfirmed(request, "TOOL_STOP_UNCONFIRMED")
                proof = ended.lease_rejection
                if proof is not None:
                    if (
                        context is None
                        or proof.instance_id != context["instance_id"]
                        or proof.profile_hash != self.profile.digest
                        or proof.epoch != context["epoch"] - 1
                    ):
                        self.stop_unconfirmed(
                            request, "TOOL_NO_GRANT_EVIDENCE_MISMATCH"
                        )
                    request["epoch"] = proof.epoch
                if (
                    self.tool_goal is not goal
                    or self.tool_result is not result
                    or self.tool_request is not context
                ):
                    self.stop_unconfirmed(request, "TOOL_GOAL_CHANGED")
            await self.stopped(request)
        except Exception as exc:
            if str(exc).startswith("PICO_STOP_UNCONFIRMED:"):
                raise
            self.stop_unconfirmed(request, type(exc).__name__ + ":" + str(exc))
        self.tool_goal = self.tool_result = None
        self.tool_phase = "inactive"
        self.tool_request = None

    async def release_lease(self):
        if self.lease is None:
            return
        if self.stop_request is not None:
            return await self.stopped(self.stop_request)
        ownership = self.lease
        lease, instance_id = ownership
        pending_grant = hasattr(lease, "done")
        request = self.begin_stop_wait(
            instance_id, -1 if pending_grant else lease.epoch, True
        )
        try:
            if pending_grant:
                lease = await self.wait(
                    lease, max(0.0, request["deadline"] - time.monotonic())
                )
                if self.lease is not ownership:
                    self.stop_unconfirmed(request, "LEASE_REQUEST_CHANGED")
                if lease.profile_hash != self.profile.digest:
                    self.stop_unconfirmed(request, "PROFILE_MISMATCH")
                request["epoch"] = lease.epoch
                if not lease.ok:
                    if lease.lease_id:
                        self.stop_unconfirmed(request, "LEASE_NOT_OWNED")
                    request["lease_active"] = False
                    await self.stopped(request)
                    self.lease = None
                    return
            reply = await self.wait(
                self.stop_client.call_async(
                    ControlExecution.Request(
                        operation="hold", lease_id=lease.lease_id, epoch=lease.epoch
                    )
                ),
                min(2.0, max(0.0, request["deadline"] - time.monotonic())),
            )
            if reply.ok:
                await self.stopped(request, clear=False, lease=lease)
            elif reply.code != "INVALID_LEASE":
                self.stop_unconfirmed(request, reply.code)
            # Keep one original deadline across hold and release. INVALID_LEASE
            # only permits checking the already-revoked next epoch below.
            request.update(
                epoch=lease.epoch + 1, lease_active=False, requested=self.now()
            )
            reply = await self.wait(
                self.leases.call_async(
                    Lease.Request(
                        operation="release", lease_id=lease.lease_id, epoch=lease.epoch
                    )
                ),
                min(2.0, max(0.0, request["deadline"] - time.monotonic())),
            )
            if not reply.ok and not (
                reply.code == "INVALID_LEASE"
                and not reply.lease_id
                and reply.epoch == lease.epoch + 1
                and reply.profile_hash == self.profile.digest
            ):
                self.stop_unconfirmed(request, reply.code)
            await self.stopped(request)
        except Exception as exc:
            if str(exc).startswith("PICO_STOP_UNCONFIRMED:"):
                raise
            self.stop_unconfirmed(request, type(exc).__name__ + ":" + str(exc))
        if self.lease is ownership:
            self.lease = None

    async def reset_grasp_reference(self, intent):
        self.current(intent)
        request = SimpleNamespace(
            execution_instance_id=self.state.instance_id, epoch=self.state.lease_epoch
        )
        reply = self.tool.reset_grasp_reference(request, SimpleNamespace())
        if not reply.ok:
            raise RuntimeError(reply.code)

    async def prepare_hold_reference(
        self, intent, *, lease=None, previous=None, allow_feedback_fault=False
    ):
        return await preparation.prepare_hold_reference(
            self,
            intent,
            lease=lease,
            previous=previous,
            allow_feedback_fault=allow_feedback_fault,
        )

    async def prepare_primitive(self, intent):
        return await preparation.prepare_primitive(self, intent)

    async def move(
        self,
        intent,
        targets,
        duration,
        *,
        fit_duration=False,
        limit_exit=False,
        replay=None
    ):
        return await preparation.move(
            self,
            intent,
            targets,
            duration,
            fit_duration=fit_duration,
            limit_exit=limit_exit,
            replay=replay,
        )

    async def validate_prepare(self, intent, targets=None, *, allow_recovery=True):
        return await preparation.validate_prepare(
            self, intent, targets, allow_recovery=allow_recovery
        )

    async def confirm_tool(self, intent, mode="bimanual_tool"):
        f = self.feedback()
        if not f.healthy:
            raise RuntimeError("PICO_FEEDBACK_UNAVAILABLE")
        if not f.stopped:
            raise RuntimeError("PICO_MODE_REQUIRES_STOPPED")
        if mode == "bimanual_tool" and self.position_contract and self.operator_grasp:
            self.operator_grasp_positions = {}
            self.control.virtual = False
        elif mode == "bimanual_tool" and self.unheld_test:
            if self.control.load == "held":
                raise RuntimeError("PICO_UNHELD_TEST_WITH_HELD_LOAD")
            self.operator_grasp_positions = {}
            self.control.virtual = False
            self.event(
                "PICO_UNHELD_TEST_READY",
                json.dumps(
                    dict(
                        operation=intent.operation,
                        measured_grasp_policy="unheld_test",
                        physical_grasp_confirmed=False,
                        grasp_sensor_confirmed=False,
                        reference_source="measured_relative",
                    )
                ),
            )
        elif mode == "bimanual_tool" and self.operator_grasp:
            # Explicit tool-mode selection / two-hand engagement confirms the
            # physical grip. Keep raw modes unknown; never invent calibration.
            states = {
                side: self.position_ops.reading(side) for side in ("left", "right")
            }
            self.operator_grasp_positions = {
                side: s.measured_position for side, s in states.items()
            }
            self.control.virtual = False
            self.event(
                "PICO_TOOL_GRASP_OPERATOR",
                json.dumps(
                    dict(
                        operation=intent.operation,
                        gripper_positions=self.operator_grasp_positions,
                        grasp_sensor_confirmed=False,
                        reference_source=self.tool_cfg.get(
                            "grasp_reference_source", "measured_rigid"
                        ),
                    )
                ),
            )
        elif (
            mode == "bimanual_tool"
            and self.position_ops
            and not self.position_ops.config["calibrated"]
        ):
            raise RuntimeError("PICO_TOOL_GRASP_UNCONFIRMED_UNCALIBRATED")
        if (
            mode == "bimanual_tool"
            and not self.position_contract
            and not getattr(self, "independent_grippers", False)
            and not self.unheld_test
            and not self.operator_grasp
            and (f.modes[0] != "closed" or f.modes[1] not in ("closed", "virtual"))
        ):
            raise RuntimeError("PICO_GRIPPER_CONFIRMATION_REQUIRED")
        end = time.monotonic() + self.tool_cfg.get("worker_startup_timeout", 30.0)
        while time.monotonic() < end:
            self.current(intent)
            reply = self.tool.preflight(SimpleNamespace(), mode)
            if reply.success:
                self.current(intent)
                if (
                    not self.position_contract
                    and mode == "bimanual_tool"
                    and self.operator_grasp
                    and not self.unheld_test
                    and self.control.load != "held"
                ):
                    # Invalidate older context declarations. The in-flight
                    # operation remains fenced by its unchanged pending token,
                    # independently of this context generation.
                    self.control.load, self.control.load_source = (
                        "held",
                        "operator_tool_confirmation",
                    )
                    self.control.context_at = self.now()
                    self.control.generation += 1
                    self.event(
                        "PICO_LOAD_OPERATOR_CONFIRMED",
                        json.dumps(
                            dict(
                                load_state="held",
                                load_source=self.control.load_source,
                                generation=self.control.generation,
                                operation=intent.operation,
                                physical_grasp_confirmed_by="operator",
                                grasp_sensor_confirmed=False,
                            )
                        ),
                    )
                return
            if reply.message not in (
                "BIMANUAL_WORKER_NOT_READY",
                "BIMANUAL_PREFLIGHT_PENDING",
            ):
                raise RuntimeError(reply.message)
            await self.pause()
        raise RuntimeError("PICO_TOOL_CONFIRM_TIMEOUT")

    async def select_tool_mode(self, intent, mode=None):
        self.current(intent)
        mode = mode or self.control.teleop_mode
        self.release_until = float("inf")
        changed = self.selection.begin(mode, "", self.selection.generation, self.now())
        self.publish_mode()
        if changed:
            deadline = time.monotonic() + 5.0
            reason = "MODE_SWITCH_TIMEOUT"
            try:
                while time.monotonic() < deadline:
                    self.current(intent)
                    s = self.state
                    view = (
                        None
                        if s is None
                        else SimpleNamespace(
                            profile_hash=s.profile_hash,
                            lease_active=s.lease_active,
                            state=s.state,
                            reference=s.reference,
                            stop_failures=s.stop_failures,
                            feedback_stamp=seconds(s.feedback_stamp),
                            joints=s.joints,
                        )
                    )
                    health = self.tool.health
                    health_view = (
                        None
                        if health is None
                        else SimpleNamespace(
                            ready=health.ready, stamp=seconds(health.stamp)
                        )
                    )
                    reason = self.selection.ready(
                        view,
                        health_view,
                        self.tool.frames,
                        self.now(),
                        tuple(self.profile.groups),
                    )
                    if not reason:
                        ready = self.tool.preflight(SimpleNamespace(), mode)
                        reason = "" if ready.success else ready.message
                    if not reason:
                        self.selection.finish(self.now())
                        self.publish_mode()
                        break
                    if reason == "BIMANUAL_GRASP_GEOMETRY" or reason.startswith(
                        (
                            "BIMANUAL_WORKER_FAILED",
                            "BIMANUAL_WORKER_TIMEOUT",
                            "BIMANUAL_WORKER_EXITED",
                        )
                    ):
                        raise RuntimeError(reason)
                    await self.pause()
                else:
                    raise RuntimeError("MODE_SWITCH_TIMEOUT:" + reason)
            except Exception as exc:
                self.selection.fail(str(exc), self.now())
                self.publish_mode()
                raise
        # Observe fresh projected input after the authority changed.
        while self.now() - seconds(self.mode.changed_at) <= 0.06:
            self.current(intent)
            await self.pause()

    async def follow(self, intent):
        self.current(intent)
        if not self.feedback().stopped:
            raise RuntimeError("PICO_MOTION_REQUIRES_STOP")
        state, _ = self.reading()
        if state.lease_active:
            raise RuntimeError("PICO_EXECUTION_AUTHORITY_CHANGED")
        context = dict(
            instance_id=state.instance_id,
            epoch=state.lease_epoch + 1,
            token=intent.token,
            canceling=False,
            future=None,
        )
        self.tool_request = context
        self.release_until = self.now() + 0.1
        request = TeleopSession.Goal(
            task_id="pico_" + self.instance_id,
            resource_group=self.control.teleop_mode,
            duration=3600.0,
        )
        if self.tool.goal(request) != GoalResponse.ACCEPT:
            self.tool_request = None
            raise RuntimeError("PICO_TOOL_SESSION_REJECTED")
        operation = SessionOperation(
            request, lambda feedback: self.tool_feedback(feedback, context)
        )
        self.tool_goal = operation
        self.tool_result = self.executor.create_task(self.tool.execute(operation))
        self.current(intent)
        end = time.monotonic() + 5.0
        while time.monotonic() < end:
            self.current(intent)
            if self.tool_result.done():
                raise RuntimeError(self.tool_result.result().code)
            if self.tool_phase == "follow" or self.tool_phase.startswith("follow:"):
                return
            await self.pause()
        raise RuntimeError("PICO_FOLLOW_CAPTURE_TIMEOUT")

    def tool_feedback(self, msg, context):
        if context is self.tool_request and not context["canceling"]:
            self.tool_phase = msg.mode

    async def recover(self, intent):
        """B rechecks current stop evidence; it never retries motion or enable."""
        self.current(intent)
        if (
            self.lease is not None
            or self.tool_request is not None
            or self.tool_goal is not None
        ):
            raise RuntimeError("PICO_RECOVERY_OWNERSHIP_UNRESOLVED")
        s, _ = self.reading()
        pending = self.stop_request
        request = pending or getattr(self, "confirmed_stop", None)
        feedback_fault = s.state == "FAILED" and s.code == "DEVICE_FEEDBACK_ERROR"
        if request is not None:
            matches = not request["lease_active"] and stop_feedback_matches(
                s,
                instance_id=request["instance_id"],
                epoch=request["epoch"],
                lease_active=False,
                profile_hash=self.profile.digest,
                simulated=self.simulation,
                requested=request["requested"],
                now=self.now(),
                max_age=self.tool_cfg["feedback_max_age"],
                at_rest=lambda joints: joints_at_rest(
                    joints,
                    self.names,
                    self.profile.feedback_tolerances.get("joint_velocity", 0.01),
                ),
            )
        else:
            matches = (
                not s.lease_active
                and s.state in ("READY", "CANCELED", "SUCCEEDED")
                and not s.reference.name
                and joints_at_rest(
                    s.joints,
                    self.names,
                    self.profile.feedback_tolerances.get("joint_velocity", 0.01),
                )
            )
        # Only a completed stop can recover a transient feedback failure. A
        # fresh idle display alone cannot authorize FAILED or erase a barrier.
        if (
            not matches
            or s.state not in ("READY", "CANCELED", "SUCCEEDED")
            and not (
                feedback_fault
                and request is not None
                and not request["failed"]
                and pending is None
            )
        ):
            raise RuntimeError("PICO_RECOVERY_STOP_NOT_CONFIRMED")
        hold = None
        if feedback_fault:
            hold = await self.prepare_hold_reference(intent, allow_feedback_fault=True)
        if self.position_ops:
            await self.position_ops.recover(intent)
        if hold is not None:
            await self.prepare_hold_reference(
                intent, previous=hold, allow_feedback_fault=True
            )
        self.current(intent)
        # Recheck after the service await; a changed owner or fault invalidates recovery.
        current, _ = self.reading()
        if (
            self.stop_request is not pending
            or current.instance_id != s.instance_id
            or current.lease_epoch != s.lease_epoch
            or current.lease_active
            or current.state != s.state
            or current.code != s.code
            or current.stop_failures
            or current.reference.name
            or not joints_at_rest(
                current.joints,
                self.names,
                self.profile.feedback_tolerances.get("joint_velocity", 0.01),
            )
        ):
            raise RuntimeError("PICO_RECOVERY_STATE_CHANGED")
        self.stop_request = None
        self.control.update_feedback(self.feedback(), self.now())
        if not self.control.rest(self.now()):
            self.stop_request = pending
            raise RuntimeError("PICO_RECOVERY_FEEDBACK_NOT_READY")
        self.handoff_complete = True

    async def perform(self, intent):
        ok = False
        reason = ""
        read_only = False
        rejected = False
        canceled = False
        stopped_failure = False
        try:
            if intent.operation == "recover":
                await self.recover(intent)
            else:
                await self.cancel_tool()
            self.current(intent)
            read_only = intent.operation in (
                "prepare",
                "confirm_tool",
                "follow",
                "select_mode",
            )
            if (
                not self.position_contract
                and getattr(self, "automatic_close_grippers", False)
                and not getattr(self, "independent_grippers", False)
                and not self.unheld_test
                and (
                    intent.operation == "confirm_tool"
                    or intent.operation == "select_mode"
                    and intent.value == "bimanual_tool"
                )
            ):
                read_only = False
            if intent.operation == "prepare":
                # A targets the configured ready pose; held-node returns use the certified graph.
                if not self.position_contract and self.control.load != "empty":
                    raise RuntimeError("PICO_PREPARE_REQUIRES_EMPTY")
                if getattr(self, "primitive_library", None):
                    read_only = False
                    primitive_done = await self.prepare_primitive(intent)
                else:
                    primitive_done = False
                if not primitive_done:
                    target = await self.validate_prepare(intent)
                    if getattr(self, "prepare_limit_reference", None) is not None:
                        read_only = False
                        self.event(
                            "PICO_PREPARE_LIMIT_EXIT", json.dumps(dict(target=target))
                        )
                        await self.move(
                            intent,
                            target,
                            min(3.0, self.cfg["prepare_duration"]),
                            limit_exit=True,
                        )
                        self.current(intent)
                        target = await self.validate_prepare(intent)
                        if self.prepare_limit_reference is not None:
                            raise RuntimeError("PREPARE_LIMIT_RECOVERY_NOT_INSIDE")
                    _, measured = self.reading()
                    ready = measured
                    if getattr(self, "execution_mode", "position") == "impedance":
                        ready = (await self.prepare_hold_reference(intent))["positions"]
                    error = max(abs(ready[n] - q) for n, q in target.items())
                    if (
                        self.position_contract
                        and error < self.prepare_position_tolerance
                        and not self.prepare_recovery
                        and (
                            not getattr(self, "prepare_limit_reference", None)
                            or all(
                                self.profile.limits[n][0]
                                <= measured[n]
                                <= self.profile.limits[n][1]
                                for n in target
                            )
                        )
                    ):
                        self.event(
                            "PICO_PREPARE_ALREADY_REACHED",
                            json.dumps(
                                dict(
                                    max_joint_error_rad=error,
                                    position_tolerance_rad=self.prepare_position_tolerance,
                                )
                            ),
                        )
                        await self.reset_grasp_reference(intent)
                    else:
                        read_only = False
                        recovery = self.prepare_recovery
                        if recovery:
                            escape = recovery["exit_waypoint"]
                            distance = max(
                                abs(measured[n] - v) for n, v in escape.items()
                            )
                            # Rest-to-rest quintic peak speed is 1.875 * travel / time.
                            duration = max(
                                1.0, 2.5 * distance / recovery["max_joint_speed_rad_s"]
                            )
                            await self.move(intent, escape, duration)
                            self.current(intent)
                            target = await self.validate_prepare(
                                intent, allow_recovery=False
                            )
                        await self.move(
                            intent,
                            target,
                            self.cfg["prepare_duration"],
                            fit_duration=self.simulation_demo,
                        )
                    if (
                        getattr(self, "automatic_close_grippers", False)
                        and not getattr(self, "independent_grippers", False)
                        and not self.unheld_test
                    ):
                        read_only = False
                        await self.position_ops.close_grippers(intent)
                        self.current(intent)
            elif intent.operation == "manual_grippers":
                if (
                    not self.position_ops
                    or self.control.phase != "gripper_control"
                    or not self.control.frames
                    or any(
                        f.grip >= parameters_for(self, "pico")["grip_release_threshold"]
                        for f in self.control.frames
                    )
                ):
                    raise RuntimeError("GRIPPER_REQUIRES_STOP_AND_RELEASED_SQUEEZE")
                await self.position_ops.move_grippers(intent, direct=True)
            elif intent.operation == "grippers":
                if not self.control.supported or self.control.phase != "loading":
                    raise RuntimeError("PICO_LOADING_REQUIRED")
                if self.position_ops:
                    await self.position_ops.move_grippers(intent)
                else:
                    target = {
                        self.cfg["grippers"][side]["joint"]: self.cfg["grippers"][side][
                            mode
                        ]
                        for side, mode in zip(("left", "right"), intent.value)
                    }
                    await self.move(intent, target, self.cfg["gripper_duration"])
            elif intent.operation == "confirm_tool":
                await self.confirm_tool(intent)
            elif intent.operation == "select_mode":
                self.event(
                    "PICO_MODE_CHECK",
                    json.dumps(
                        dict(
                            token=intent.token,
                            current_mode=self.control.teleop_mode,
                            target_mode=intent.value,
                            source_id=self.control.source,
                            seq=self.control.seq,
                        )
                    ),
                )
                await self.confirm_tool(intent, intent.value)
                await self.select_tool_mode(intent, intent.value)
            elif intent.operation == "switch_grasp":
                if self.position_contract:
                    if (
                        getattr(self.control, "continuous_axial", False)
                        or self.unheld_test
                        or self.operator_grasp
                        or not self.position_ops.axial_available
                    ):
                        raise RuntimeError("PICO_AXIAL_GRASP_UNAVAILABLE")
                    target = self.position_ops.grasp_target(intent.value)
                    await self.validate_prepare(intent, target)
                    await self.position_ops.switch_grasp(intent)
                    self.current(intent)
                    await self.confirm_tool(intent)
                else:
                    if (
                        self.control.load != "held"
                        or self.feedback().modes[0] != "closed"
                    ):
                        raise RuntimeError("PICO_LEFT_SUPPORT_REQUIRED")
                    cfg = self.cfg["grippers"]["right"]
                    target = {
                        cfg["joint"]: cfg["virtual" if intent.value else "closed"]
                    }
                    await self.validate_prepare(intent, target)
                    await self.move(
                        intent, target, self.cfg["gripper_duration"], fit_duration=True
                    )
            elif intent.operation == "follow":
                if self.position_contract:
                    self.event(
                        "PICO_ENGAGE_CHECK",
                        json.dumps(
                            dict(
                                token=intent.token,
                                source_id=self.control.source,
                                seq=self.control.seq,
                                load_state=self.control.load,
                                feedback_stamp=self.feedback().stamp,
                                requested_mode=self.control.teleop_mode,
                                mode=self.mode.mode if self.mode else None,
                            )
                        ),
                    )
                    await self.confirm_tool(intent, self.control.teleop_mode)
                    self.current(intent)
                await self.select_tool_mode(intent)
                read_only = False
                await self.follow(intent)
            elif intent.operation not in ("stop", "recover"):
                raise RuntimeError("PICO_OPERATION_UNSUPPORTED")
            ok = True
        except Superseded:
            # A newer fenced intent owns the workflow. Keep the same measured
            # stop barrier; a cleanup failure remains a real operation error.
            reason = "PICO_OPERATION_SUPERSEDED"
            try:
                await self.cancel_tool()
                canceled = True
            except Exception as stop_error:
                reason += ":" + type(stop_error).__name__ + ":" + str(stop_error)
        except Exception as exc:
            reason = type(exc).__name__ + ":" + str(exc)
            ended = (
                self.tool_result.result()
                if self.tool_result is not None and self.tool_result.done()
                else None
            )
            capture_not_ready = (
                intent.operation == "follow"
                and ended is not None
                and str(exc) == ended.code
                and temporary_capture_refusal(ended)
            )
            try:
                if intent.operation != "recover":
                    await self.cancel_tool()
                rejected = read_only
                stopped_failure = capture_not_ready or (
                    intent.operation == "prepare"
                    and reason.startswith(
                        "RuntimeError:PICO_ACTUATOR_FEEDBACK_TIMEOUT:"
                    )
                )
            except Exception as stop_error:
                reason += ":" + type(stop_error).__name__ + ":" + str(stop_error)
        if ok and intent.operation == "manual_grippers":
            # The release barrier has just observed fresh device hold. Read it
            # now rather than depending on the coordinator timer's previous
            # feedback sample when handing a held A request back to standby.
            self.control.update_feedback(self.feedback(), self.now())
        if rejected or stopped_failure:
            self.control.update_feedback(self.feedback(), self.now())
        self.control.complete(
            intent.token,
            ok,
            reason,
            self.now(),
            rejected=rejected,
            stopped_failure=stopped_failure,
        )
        if canceled:
            self.event(
                "PICO_OPERATION_CANCELED",
                json.dumps(
                    dict(
                        token=intent.token,
                        operation=intent.operation,
                        outcome="superseded",
                        reason=reason,
                        simulated_actuator=self.simulation,
                    )
                ),
            )
        else:
            self.event(
                "PICO_OPERATION_RESULT",
                json.dumps(
                    dict(
                        token=intent.token,
                        operation=intent.operation,
                        success=ok,
                        reason=reason,
                        read_only_rejection=rejected,
                        simulated_actuator=self.simulation,
                    )
                ),
            )

    def tick(self):
        self.tool.poll_worker()
        if self.now() - self.last_mode_status >= 0.05:
            self.publish_mode()
        tick_at = time.monotonic()
        if self.last_tick_at is not None and tick_at - self.last_tick_at > 0.1:
            self.event(
                "PICO_COORDINATOR_TICK_GAP",
                json.dumps(
                    dict(
                        interval_s=tick_at - self.last_tick_at,
                        previous_tick_work=getattr(self, "tick_work", {}),
                        projected_publish_s=getattr(self, "projected_publish_s", None),
                        input_age_s=(
                            self.now() - self.control.stamp
                            if math.isfinite(self.control.stamp)
                            else None
                        ),
                        phase=self.control.phase,
                        seq=self.control.seq,
                    )
                ),
            )
        self.last_tick_at = tick_at
        self.tick_work = {}
        for f in tuple(self.waiters):
            if not f.done():
                f.set_result(None)
        try:
            item = self.validator.poll()
            if item is not None:
                self.validation_result = item
        except RuntimeError as exc:
            if self.control.pending and self.control.pending.operation == "prepare":
                self.control.lock(str(exc), self.now())
        feedback = self.feedback()
        if (
            self.state
            and not self.state.lease_active
            and feedback.healthy
            and feedback.stopped
        ):
            self.handoff_complete = True
        self.control.update_feedback(feedback, self.now())
        self.control.tick(self.now())
        if self.control.context_at != self.last_context_at:
            self.last_context_at = self.control.context_at
            self.event("PICO_CONTEXT", json.dumps(self.control.snapshot(self.now())))
        if (
            self.tool_result
            and self.tool_result.done()
            and self.control.phase in ("engaging", "following")
        ):
            ended = self.tool_result.result()
            # The pending follow operation owns cancellation and exact stop proof.
            # Do not let this timer latch its transient pre-target refusal first.
            canceling_capture = (
                self.control.pending
                and self.control.pending.operation == "follow"
                and temporary_capture_refusal(ended)
            )
            if not canceling_capture:
                prefix = (
                    "PICO_ARMS_SESSION_ENDED:"
                    if self.control.teleop_mode == "independent_arms"
                    else "PICO_TOOL_SESSION_ENDED:"
                )
                self.control.lock(prefix + ended.code, self.now())
        for intent in self.control.drain():
            if intent.operation == "record":
                if not self.recording_enabled:
                    self.event("PICO_RECORDING", "RECORDING_DISABLED")
                elif (
                    self.record_future is None and self.record_client.service_is_ready()
                ):
                    self.record_future = self.record_client.call_async(
                        SetBool.Request(data=not self.recording)
                    )
                continue
            self.pending_intent = intent
        if self.record_future and self.record_future.done():
            reply = self.record_future.result()
            if reply.success:
                self.recording = reply.message == "RECORDING_ENABLED"
            self.event("PICO_RECORDING", reply.message)
            self.record_future = None
        if self.running is not None and self.running.done():
            self.running = None
        if self.running is None and self.pending_intent is not None:
            intent, self.pending_intent = self.pending_intent, None
            self.running = self.executor.create_task(self.perform(intent))
        if self.base:
            base_started = time.monotonic()
            self.base.tick(self.control, feedback.healthy)
            self.tick_work["base_s"] = time.monotonic() - base_started
            self.tick_work["base_detail"] = getattr(self.base, "tick_work", {})
        if self.now() - self.last_status >= 0.1:
            self.last_status = self.now()
            if self.simulation_demo:
                try:
                    _, q = self.reading()
                    ready = self.prepare_positions
                    prepared = all(
                        abs(q[n] - ready[n]) < 0.003
                        for g in self.tool_cfg["resource_groups"]
                        for n in self.profile.groups[g]
                    )
                    self.control.simulation_fixture(prepared, self.now())
                except RuntimeError:
                    pass
            view = operator_view(self, feedback)
            publish_started = time.monotonic()
            serialized = json.dumps(view)
            self.operator_state.publish(String(data=serialized))
            self.tick_work["state_publish_s"] = time.monotonic() - publish_started
            if self.last_phase != (self.control.phase, self.control.reason):
                self.last_phase = (self.control.phase, self.control.reason)
                self.event("PICO_STATE", serialized)
        self.tick_work["total_s"] = time.monotonic() - tick_at

    def input_status(self, now):
        """Display the coordinated pair without hiding raw tracking loss."""
        raw = tuple(self.frames.values())
        pair = self.control.frames or ()
        ages = [now - seconds(f.stamp) for f in raw]
        tracking = {
            f.side: bool(f.valid and 0 <= age <= self.control.max_age)
            for f, age in zip(raw, ages)
        }
        same_source = bool(
            len(raw) == len(pair) == 2
            and all(f.source_id == pair[0].source_id for f in raw)
        )
        valid = bool(
            same_source
            and all(tracking.values())
            and all(
                f.valid and 0 <= now - f.stamp <= self.control.max_age for f in pair
            )
            and all(f.seq >= pair[0].seq for f in raw)
        )
        return dict(
            tracking_valid=tracking,
            raw_input_age_s=max(ages, default=None),
            raw_input_valid=valid,
            input_pair_pending=bool(valid and any(f.seq != pair[0].seq for f in raw)),
        )

    def close(self):
        if self.base:
            self.base.close()
        self.validator.close()
        self.tool.close()
