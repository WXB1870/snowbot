"""Snowbot preparation paths, validation and measured arrival.

The application supplies its current operation, execution transport and stop
barrier. This module owns preparation geometry and target generation.
"""

import json
import time
import uuid
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from bindu_interfaces.msg import MotionCommand
from bindu_interfaces.srv import Lease, SubmitMotion
from bindu_execution.interpolation import Curve, State
from bindu_runtime.common import seconds, stamp, joints_at_rest
from snowbot_hardware.parameters import parameters_for
from .control import prepare_reference, prepare_start_tolerance


async def prepare_primitive(app, intent):
    """A return from a certified held-tool node, using the same transition graph."""
    from snowbot_bringup.teleop.snowbot_primitive_replay import plan_from_current

    app.current(intent)
    state, measured = app.reading()
    if not joints_at_rest(
        state.joints,
        app.names,
        parameters_for(app, "execution")["prepare_start_velocity_tolerance_rad_s"],
    ):
        raise RuntimeError("PREPARE_REQUIRES_STATIONARY_START")
    try:
        plan = plan_from_current(app.primitive_library, "Home", measured)
    except RuntimeError as exc:
        if (
            str(exc).startswith("REPLAY_UNKNOWN_START")
            and app.control.teleop_mode == "independent_arms"
            and (
                app.control.load == "empty"
                or app.unheld_test
                or app.feedback().modes == ("open", "open")
            )
        ):
            # Ordinary-arm A remains an explicit empty-hand setup request.
            return False
        raise
    app.prepare_limit_reference = app.prepare_limit_start = app.prepare_recovery = None
    app.event(
        "PICO_PRIMITIVE_HOME",
        json.dumps(
            dict(
                route=list(plan.route), duration_s=plan.duration, transition_validation_passed=True
            )
        ),
    )
    await app.move(intent, dict(zip(plan.names, plan.points[-1])), plan.duration, replay=plan)
    return True


async def move(
    app, intent, targets, duration, *, fit_duration=False, limit_exit=False, replay=None
):
    app.current(intent)
    if not app.feedback().stopped:
        raise RuntimeError("PICO_MOTION_REQUIRES_STOP")
    arrival_tolerance = (
        app.prepare_position_tolerance
        if intent.operation == "prepare"
        else app.profile.feedback_tolerances["joint_position"]
    )
    execution_instance_id = app.state.instance_id
    future = app.leases.call_async(
        Lease.Request(operation="acquire", owner=app.instance_id, resources=list(app.groups))
    )
    app.lease = (future, execution_instance_id)
    try:
        lease = await app.wait(future)
        if not lease.ok or lease.profile_hash != app.profile.digest:
            raise RuntimeError(lease.code if not lease.ok else "PROFILE_MISMATCH")
    except Exception:
        # A lost grant reply is not a rejected grant. Cleanup settles that
        # exact future before releasing, or retains an unconfirmed barrier.
        await app.release_lease()
        raise
    app.lease = (lease, execution_instance_id)
    try:
        app.current(intent)
        deadline = time.monotonic() + 1.0
        while not (
            app.state
            and app.state.instance_id == execution_instance_id
            and app.state.lease_active
            and app.state.lease_epoch == lease.epoch
        ):
            app.current(intent)
            if time.monotonic() > deadline:
                raise RuntimeError("PICO_LEASE_FEEDBACK_TIMEOUT")
            await app.pause()
        s, q = app.reading()
        end = [targets.get(n, q[n]) for n in app.names]
        recovery = (
            getattr(app, "prepare_limit_reference", None) if intent.operation == "prepare" else None
        )
        recovery_progress = {}
        if recovery is not None:
            from bindu_kinematics.bimanual_worker import prepare_limit_reference

            held = await app.prepare_hold_reference(intent, lease=lease, previous=recovery)
            s, q = app.reading()
            if not joints_at_rest(
                s.joints,
                app.names,
                parameters_for(app, "execution")["prepare_start_velocity_tolerance_rad_s"],
            ):
                raise RuntimeError("PREPARE_REQUIRES_STATIONARY_START")
            changed = max(app.names, key=lambda n: abs(q[n] - app.prepare_limit_start[n]))
            tolerance = prepare_start_tolerance(app.profile, app.cfg)
            if abs(q[changed] - app.prepare_limit_start[changed]) > tolerance:
                app.event(
                    "PICO_PREPARE_START_CHANGED",
                    json.dumps(
                        dict(
                            joint=changed,
                            before_rad=app.prepare_limit_start[changed],
                            after_rad=q[changed],
                            delta_rad=q[changed] - app.prepare_limit_start[changed],
                            tolerance_rad=tolerance,
                            before_feedback_stamp=getattr(app, "prepare_limit_start_stamp", None),
                            after_feedback_stamp=(
                                seconds(s.feedback_stamp) if hasattr(s, "feedback_stamp") else None
                            ),
                        )
                    ),
                )
                raise RuntimeError("PREPARE_STATE_CHANGED")
            start = prepare_limit_reference(
                app.profile,
                app.names,
                q,
                held["positions"],
                targets,
                app.prepare_limit_max,
                app.prepare_position_tolerance,
            )
            recovery_progress = {
                n: (-1 if q[n] < app.profile.limits[n][0] else 1, q[n])
                for n in app.names
                if not app.profile.limits[n][0] <= q[n] <= app.profile.limits[n][1]
            }
            # Reuse the existing conservative collision-escape speed cap.
            if recovery_progress:
                speed = app.cfg.get("prepare_collision_recovery", {}).get(
                    "max_joint_speed_rad_s", 0.05
                )
                duration = max(
                    duration, 2.5 * max(abs(start[n] - v) for n, v in zip(app.names, end)) / speed
                )
        else:
            start = (
                prepare_reference(app.profile, app.names, q)
                if app.position_contract and intent.operation == "prepare"
                else q
            )
        if limit_exit and not recovery_progress:
            raise RuntimeError("PREPARE_LIMIT_RECOVERY_HOLD_REQUIRED")
        arrival_names = set(recovery_progress) if limit_exit else set(app.names)
        revision = s.revision
        segment = 0
        if replay is not None:
            if (
                tuple(app.names) != replay.names
                or max(abs(q[n] - x) for n, x in zip(app.names, replay.points[0])) > 0.01
            ):
                raise RuntimeError("PREPARE_STATE_CHANGED")
        elif getattr(app, "primitive_library", None) and intent.operation == "prepare":
            travel = max(
                abs(end[i] - start[n])
                for i, n in enumerate(app.names)
                if n.startswith("body_joint_")
            )
            duration = max(duration, 1.875 * travel / 0.025, (5.774 * travel / 0.12) ** 0.5)
        before = State.rest(tuple(start[n] for n in app.names))
        after = State.rest(tuple(end))
        curve = None if replay is not None else Curve.between(0.0, duration, before, after)
        if replay is None and not curve.valid(app.profile, app.names):
            raise RuntimeError("PICO_MOTION_LIMITS")
        if fit_duration and replay is None:
            # Fit simulation-demo preparation and small grasp changes to
            # the same v/a/jerk limits. Loading/unloading and the strict
            # preparation flow retain their configured duration.
            # Keep transport reserve beyond the 120 ms reference horizon;
            # fitting the mathematical curve to its exact jerk boundary
            # leaves no budget for timestamp/service scheduling jitter.
            while duration * 0.75 >= 0.3:
                candidate = Curve.between(0.0, duration * 0.75, before, after)
                if not candidate.valid(app.profile, app.names):
                    break
                duration *= 0.75
                curve = candidate
        app.event(
            "PICO_ACTUATOR_STARTED",
            json.dumps(
                dict(
                    operation=intent.operation,
                    duration_s=duration,
                    position_tolerance_rad=arrival_tolerance,
                    fit_duration=fit_duration,
                    simulated_actuator=app.simulation,
                )
            ),
        )
        started = app.now()
        command_id = app.instance_id + "_" + str(intent.token) + "_" + uuid.uuid4().hex[:8]
        deadline = (
            time.monotonic() + duration + parameters_for(app, "pico")["motion_completion_grace_s"]
        )
        renew_at = 0.0
        next_submit = 0.0
        renewal = None
        renewal_started = 0.0
        replay_tick = 0
        replay_steady = time.monotonic()
        while time.monotonic() < deadline:
            app.current(intent)
            state, measured = app.reading()
            now = app.now()
            if replay is not None:
                if (
                    abs((now - started) - (time.monotonic() - replay_steady))
                    > parameters_for(app, "pico")["prepare_command_ttl_s"]
                ):
                    raise RuntimeError("REPLAY_CLOCK_CHANGED")
            if (
                state.instance_id == execution_instance_id
                and state.command_id.startswith(command_id + "_")
                and state.state in ("STOPPING", "FAILED")
                and state.code not in ("OK", "CANCELED", "SHUTDOWN")
            ):
                raise RuntimeError(state.code)
            if (
                state.instance_id != execution_instance_id
                or not state.lease_active
                or state.lease_epoch != lease.epoch
            ):
                raise RuntimeError("PICO_EXECUTION_AUTHORITY_CHANGED")
            for n, (side, best) in tuple(recovery_progress.items()):
                if side * (measured[n] - best) > app.profile.feedback_tolerances["joint_position"]:
                    raise RuntimeError("PREPARE_LIMIT_RECOVERY_MOVED_OUTWARD:" + n)
                recovery_progress[n] = (
                    side,
                    min(best, measured[n]) if side > 0 else max(best, measured[n]),
                )
            if renewal is not None:
                if (
                    time.monotonic() - renewal_started
                    >= parameters_for(app, "pico")["renewal_timeout_s"]
                ):
                    raise RuntimeError("PICO_LEASE_RENEW_TIMEOUT")
                if renewal.done():
                    reply = renewal.result()
                    if not reply.ok:
                        raise RuntimeError(reply.code)
                    if (
                        reply.lease_id != lease.lease_id
                        or reply.epoch != lease.epoch
                        or reply.profile_hash != app.profile.digest
                    ):
                        raise RuntimeError("PICO_EXECUTION_AUTHORITY_CHANGED")
                    renewal = None
                    renew_at = time.monotonic() + parameters_for(app, "pico")["renewal_period_s"]
            if (
                now >= started + duration
                and (replay is None or (replay_tick - 1) * 0.02 >= duration)
                and seconds(state.feedback_stamp) >= started + duration
                and all(
                    abs(measured[n] - v) < arrival_tolerance
                    for n, v in zip(app.names, end)
                    if n in arrival_names
                )
                and (
                    recovery is None
                    or all(
                        app.profile.limits[n][0] <= measured[n] <= app.profile.limits[n][1]
                        for n in app.names
                    )
                )
                and (
                    not limit_exit
                    or all(
                        app.profile.limits[n][0] + app.profile.feedback_tolerances["joint_position"]
                        <= measured[n]
                        <= app.profile.limits[n][1]
                        - app.profile.feedback_tolerances["joint_position"]
                        for n in recovery_progress
                    )
                )
                and joints_at_rest(
                    state.joints,
                    app.names,
                    parameters_for(app, "execution")["motion_arrival_velocity_tolerance_rad_s"],
                )
            ):
                app.event(
                    "PICO_ACTUATOR_ARRIVED",
                    json.dumps(
                        dict(
                            operation=intent.operation,
                            position_tolerance_rad=arrival_tolerance,
                            error_rad={n: float(v - measured[n]) for n, v in zip(app.names, end)},
                            measured_at_rest=True,
                        )
                    ),
                )
                return
            if renewal is None and time.monotonic() >= renew_at:
                # A slow renewal reply must not suspend fresh target supply.
                # The executor still enforces lease expiry on every submit.
                renewal = app.leases.call_async(
                    Lease.Request(operation="renew", lease_id=lease.lease_id, epoch=lease.epoch)
                )
                renewal_started = time.monotonic()
            if now >= next_submit:
                lookahead = parameters_for(app, "pico")["reference_lookahead_s"]
                lookahead_sec, lookahead_ns = divmod(round(lookahead * 1e9), 1000000000)
                source = started + replay_tick * 0.02 if replay is not None else now
                if (
                    replay is not None
                    and now - source >= parameters_for(app, "pico")["prepare_command_ttl_s"]
                ):
                    raise RuntimeError("REPLAY_STREAM_LATE")
                sample = (
                    State.rest(replay.sample(replay_tick * 0.02))
                    if replay is not None
                    else curve.sample(
                        min(
                            duration,
                            max(0.0, now - started) + (0.0 if app.position_contract else lookahead),
                        )
                    )
                )
                point = JointTrajectoryPoint(
                    positions=list(sample.q),
                    velocities=list(sample.v),
                    accelerations=list(sample.a),
                    time_from_start=Duration(sec=lookahead_sec, nanosec=lookahead_ns),
                )
                msg = MotionCommand(
                    schema_version=2,
                    runtime_mode=app.runtime_mode,
                    command_id=command_id + "_" + str(segment),
                    task_id="pico_" + intent.operation,
                    expected_revision=revision,
                    profile_hash=app.profile.digest,
                    lease_id=lease.lease_id,
                    epoch=lease.epoch,
                    mode="joint_target" if app.position_contract else "joint_reference_segment",
                    resource_groups=list(app.groups),
                    stamp=stamp(source),
                    valid_for=(
                        0.25
                        if replay is not None
                        else parameters_for(app, "pico")["prepare_command_ttl_s"]
                    ),
                    joint_names=list(app.names),
                    positions=list(sample.q) if app.position_contract else [],
                    points=[] if app.position_contract else [point],
                )
                app.current(intent)
                response = await app.wait(app.submit.call_async(SubmitMotion.Request(command=msg)))
                if not response.accepted:
                    raise RuntimeError(response.code)
                revision = response.revision
                segment += 1
                replay_tick += 1
                next_submit = (
                    started + replay_tick * 0.02
                    if replay is not None
                    else now + parameters_for(app, "pico")["prepare_command_period_s"]
                )
            await app.pause()
        state, measured = app.reading()
        errors = {n: float(v - measured[n]) for n, v in zip(app.names, end)}
        worst = max(errors, key=lambda n: abs(errors[n]))
        app.event(
            "PICO_ACTUATOR_FEEDBACK_TIMEOUT",
            json.dumps(
                dict(
                    operation=intent.operation,
                    elapsed_s=app.now() - started,
                    feedback_stamp=seconds(state.feedback_stamp),
                    target=dict(zip(app.names, end)),
                    measured={n: measured[n] for n in app.names},
                    reference=dict(zip(state.reference.name, state.reference.position)),
                    error_rad=errors,
                    worst_joint=worst,
                    position_tolerance_rad=arrival_tolerance,
                    measured_at_rest=joints_at_rest(
                        state.joints,
                        app.names,
                        parameters_for(app, "execution")["motion_arrival_velocity_tolerance_rad_s"],
                    ),
                    velocity_tolerance_rad_s=parameters_for(app, "execution")[
                        "motion_arrival_velocity_tolerance_rad_s"
                    ],
                    arrival_confirmed=False,
                )
            ),
        )
        raise RuntimeError(f"PICO_ACTUATOR_FEEDBACK_TIMEOUT:{worst}:error_rad={errors[worst]:.6f}")
    finally:
        await app.release_lease()


async def validate_prepare(app, intent, targets=None, *, allow_recovery=True):
    app.current(intent)
    s, q = app.reading()
    start_stamp = seconds(s.feedback_stamp) if hasattr(s, "feedback_stamp") else None
    app.prepare_recovery = None
    app.prepare_limit_reference = None
    app.prepare_limit_start = None
    if targets is None and app.prepare_positions is None:
        raise RuntimeError("PICO_PREPARE_PRESET_REQUIRED")
    if not joints_at_rest(
        s.joints,
        app.names,
        (
            parameters_for(app, "execution")["prepare_start_velocity_tolerance_rad_s"]
            if app.position_contract
            else 1e-6
        ),
    ):
        raise RuntimeError("PREPARE_REQUIRES_STATIONARY_START")
    gripper_move = targets is not None
    start_tolerance = prepare_start_tolerance(app.profile, app.cfg)
    target = (
        targets
        if gripper_move
        else {
            n: app.prepare_positions[n]
            for g in app.tool_cfg["resource_groups"]
            for n in app.profile.groups[g]
        }
    )
    end = time.monotonic() + app.validator.startup_timeout
    while not app.validator.ready or app.validator.busy:
        app.current(intent)
        if time.monotonic() > end:
            raise RuntimeError("PICO_PREPARE_VALIDATOR_TIMEOUT")
        await app.pause()
    names = [n for g in app.tool_cfg["resource_groups"] for n in app.profile.groups[g]]
    reference = q
    # The held-reference recovery contract is only for measured excursions.
    # An ordinary in-bounds A return retains its original measured-start path.
    if (
        app.position_contract
        and not gripper_move
        and getattr(app, "prepare_limit_max", 0.0)
        and any(not app.profile.limits[n][0] <= q[n] <= app.profile.limits[n][1] for n in names)
    ):
        from bindu_kinematics.bimanual_worker import prepare_limit_reference

        held = await app.prepare_hold_reference(intent)
        reference = prepare_limit_reference(
            app.profile,
            names,
            q,
            held["positions"],
            target,
            app.prepare_limit_max,
            app.prepare_position_tolerance,
        )
        from bindu_kinematics.bimanual_worker import prepare_limit_exit_target

        target = prepare_limit_exit_target(
            app.profile, names, q, reference, target, app.prepare_position_tolerance
        )
        app.prepare_limit_reference = held
        app.prepare_limit_start = dict(q)
        app.prepare_limit_start_stamp = start_stamp
    elif app.position_contract and not gripper_move:
        reference = prepare_reference(app.profile, names, q)
    extra = {}
    if reference != q:
        extra["reference"] = reference
        app.event(
            "PICO_PREPARE_BOUNDARY_REFERENCE",
            json.dumps(
                dict(
                    measured={n: q[n] for n in names if q[n] != reference[n]},
                    reference={n: reference[n] for n in names if q[n] != reference[n]},
                    reference_source=(
                        "confirmed_pause" if app.prepare_limit_reference else "boundary_roundoff"
                    ),
                    tolerance_rad=(
                        app.prepare_position_tolerance
                        if app.prepare_limit_reference
                        else app.profile.feedback_tolerances["joint_position"]
                    ),
                )
            ),
        )
    app.validation_result = None
    # Holding A explicitly requests a preset return. Validate the
    # robot path without preserving a previously captured tool grasp.
    tool_present = gripper_move and app.control.teleop_mode == "bimanual_tool"
    operation = (
        ("gripper_path" if tool_present else "gripper_body_path")
        if gripper_move
        else "prepare_path"
    )
    if allow_recovery and operation == "prepare_path" and app.cfg.get("prepare_collision_recovery"):
        operation = "prepare_recovery_path"
    if app.prepare_limit_reference is not None:
        operation = "prepare_limit_recovery_path"
    app.event(
        "PICO_PREPARE_VALIDATION",
        json.dumps(
            dict(
                operation=operation,
                load_state=app.control.load,
                load_state_checked=False,
                tool_present=tool_present,
                coupling_checked=False,
                measured_grasp_policy="unheld_test" if app.unheld_test else "enforce",
            )
        ),
    )
    if not app.validator.submit(
        operation, intent.token, q, target if gripper_move else [target[n] for n in names], **extra
    ):
        raise RuntimeError("PICO_PREPARE_VALIDATOR_BUSY")
    end = time.monotonic() + 30.0
    while app.validation_result is None:
        app.current(intent)
        if time.monotonic() > end:
            raise RuntimeError("PICO_PREPARE_VALIDATION_TIMEOUT")
        await app.pause()
    kind, token, value = app.validation_result
    if token != intent.token or kind != "result":
        raise RuntimeError(str(value))
    current_state, current = app.reading()
    if not joints_at_rest(
        current_state.joints,
        app.names,
        (
            parameters_for(app, "execution")["prepare_start_velocity_tolerance_rad_s"]
            if app.position_contract
            else 1e-6
        ),
    ):
        raise RuntimeError("PREPARE_REQUIRES_STATIONARY_START")
    checked_names = app.names if gripper_move else names
    changed = max(checked_names, key=lambda n: abs(current[n] - q[n]))
    if abs(current[changed] - q[changed]) > start_tolerance:
        app.event(
            "PICO_PREPARE_START_CHANGED",
            json.dumps(
                dict(
                    joint=changed,
                    before_rad=q[changed],
                    after_rad=current[changed],
                    delta_rad=current[changed] - q[changed],
                    tolerance_rad=start_tolerance,
                    before_feedback_stamp=start_stamp,
                    after_feedback_stamp=(
                        seconds(current_state.feedback_stamp)
                        if hasattr(current_state, "feedback_stamp")
                        else None
                    ),
                )
            ),
        )
        raise RuntimeError("PREPARE_STATE_CHANGED")
    app.prepare_recovery = value.get("self_collision_recovery") if isinstance(value, dict) else None
    if app.prepare_recovery:
        app.event("PICO_PREPARE_RECOVERY", json.dumps(app.prepare_recovery))
    return target
