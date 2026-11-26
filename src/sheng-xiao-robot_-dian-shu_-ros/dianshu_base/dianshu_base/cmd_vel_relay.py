"""Connect the Nav2 controller output to the velocity smoother input."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelRelay(Node):
    """Relay controller commands while keeping the Nav2 topic boundaries explicit."""

    def __init__(self):
        """Create the raw-command subscriber and smoother-command publisher."""
        super().__init__("cmd_vel_relay")
        self.declare_parameter("input_topic", "/cmd_vel_raw")
        self.declare_parameter("output_topic", "/cmd_vel_nav")
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self._relay, 10)
        self.get_logger().info("Relaying {} -> {}".format(input_topic, output_topic))

    def _relay(self, message: Twist) -> None:
        self._publisher.publish(message)


def main(args=None):
    """Run the command relay node."""
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
