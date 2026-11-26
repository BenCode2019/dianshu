"""Small differential-drive base used to test Nav2 without the B20 MCU."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class FakeBase(Node):
    """Integrate commanded velocity and publish the odometry Nav2 expects."""

    def __init__(self):
        """Create a watchdog-limited fake base."""
        super().__init__("fake_base")
        self.declare_parameter("cmd_vel_timeout", 0.5)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self._timeout = float(self.get_parameter("cmd_vel_timeout").value)
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        if self._timeout <= 0.0:
            raise ValueError("cmd_vel_timeout must be positive")
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._linear = 0.0
        self._angular = 0.0
        self._last_command = self.get_clock().now()
        self._last_update = self.get_clock().now()
        self._odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self._cmd_vel_callback, 10)
        self._timer = self.create_timer(0.02, self._update)
        self.get_logger().warn(
            "Using fake_base: /cmd_vel is integrated locally; no motor or encoder data is used."
        )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._linear = float(msg.linear.x)
        self._angular = float(msg.angular.z)
        self._last_command = self.get_clock().now()

    def _update(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_update).nanoseconds * 1e-9
        self._last_update = now
        if dt <= 0.0 or dt > 0.2:
            dt = 0.02
        if (now - self._last_command).nanoseconds * 1e-9 > self._timeout:
            linear, angular = 0.0, 0.0
        else:
            linear, angular = self._linear, self._angular
        self._x += linear * math.cos(self._theta) * dt
        self._y += linear * math.sin(self._theta) * dt
        self._theta = math.atan2(
            math.sin(self._theta + angular * dt), math.cos(self._theta + angular * dt)
        )
        stamp = now.to_msg()
        half_theta = self._theta / 2.0
        qz, qw = math.sin(half_theta), math.cos(half_theta)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        self._odom_publisher.publish(odom)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(transform)


def main(args=None):
    """Run the fake base node."""
    rclpy.init(args=args)
    node = FakeBase()
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
