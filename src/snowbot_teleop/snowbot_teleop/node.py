"""ROS entry and transport wiring for the Snowbot teleoperation application."""

import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool
from bindu_interfaces.msg import VRInput, RecorderHealth, RuntimeEvent, TeleopModeState
from bindu_interfaces.srv import PicoContext, ResetToolReference
from bindu_runtime.common import RuntimeNode, spin
from bindu_runtime.execution_client import ExecutionClient
from bindu_runtime.tool_session import ToolSession
from .application import PicoApplication
from .tool_node import load_grippers, load_tool


class PicoNode(RuntimeNode):
    def __init__(self):
        super().__init__("pico_control", allow_hardware=True, default_mode="simulation")
        execution = ExecutionClient(self)
        app = self.application = PicoApplication(self, execution)
        tool = ToolSession(
            self,
            app.pause,
            execution=execution,
            gripper_config_loader=lambda config, profile: load_grippers(
                config, profile, self.runtime_mode
            ),
            tool_config_loader=load_tool,
        )
        app.attach_tool(tool)
        self.create_service(
            ResetToolReference,
            "teleop/tool_reference_reset",
            tool.reset_grasp_reference,
        )
        app.projected = self.create_publisher(VRInput, "teleop/vr/input", 10)
        app.gripper_projected = (
            self.create_publisher(VRInput, "teleop/vr/gripper_input", 10)
            if app.independent_grippers
            else None
        )
        app.operator_state = self.create_publisher(String, "teleop/state", 10)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        app.mode_output = self.create_publisher(
            TeleopModeState, "teleop/mode_state", qos
        )
        self.create_subscription(VRInput, "teleop/vr/raw", app.on_frame, 10)
        self.create_subscription(RuntimeEvent, "events", app.on_motion_event, 20)
        if app.recording_enabled:
            app.record_client = self.create_client(SetBool, "recorder/capture")
            self.create_subscription(
                RecorderHealth, "recorder/health", self.on_recording, 5
            )
        self.create_service(PicoContext, "teleop/pico/context", app.on_context)
        app.publish_mode()
        self.create_timer(0.005, app.tick)

    def on_recording(self, message):
        self.application.on_recording(message)
        self.application.tool.on_health(message)

    def destroy_node(self):
        self.application.close()
        return super().destroy_node()


def main():
    os.environ.setdefault("RMW_FASTRTPS_PUBLICATION_MODE", "ASYNCHRONOUS")
    spin(PicoNode, threaded=False)
