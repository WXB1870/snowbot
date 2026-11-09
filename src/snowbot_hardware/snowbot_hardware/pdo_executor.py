"""PDO take metadata for the Snowbot adapter on ROS 2 Humble."""
import time
from rclpy.executors import SingleThreadedExecutor

from .session import PdoDeliveryWindow


class HumbleObservationExecutor(SingleThreadedExecutor):
    """Keep the metadata Humble normally discards for the adapter's PDO reader."""
    def add_node(self, node):
        self.devices = node.snowbot_devices
        self.devices.pdo_delivery = PdoDeliveryWindow()
        return super().add_node(node)

    def _take_subscription(self, sub):
        if sub is not self.devices.feedback_subscription:
            return super()._take_subscription(sub)
        with sub.handle:
            result = sub.handle.take_message(sub.msg_type, sub.raw)
            taken_system_ns = time.time_ns()
        if result is not None:
            msg, info = result
            self.devices.pdo_delivery.taken = (msg, info, taken_system_ns)
            return msg
        return None
