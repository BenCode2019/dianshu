"""Verify serial telemetry becomes correctly typed ROS IMU and odometry."""

import math
import os
import pty
import time

import pytest
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from dianshu_base.base_driver import BaseDriver


IMU_FRAME = bytes.fromhex('ff 04 ff cf 00 0a 00 0d 02 58 ff 08 3e 24 fe')
WHEEL_FRAME = bytes.fromhex('ff 01 80 64 00 64 fe')


@pytest.fixture
def driver_factory():
    """Create a real ROS driver with a pseudoterminal instead of hardware."""
    rclpy.init(args=[])
    nodes = []
    descriptors = []

    def create(extra_args=None):
        master, slave = pty.openpty()
        descriptors.extend([master, slave])
        arguments = ['--ros-args', '-p', 'lower_mcu_serial_port:=' + os.ttyname(slave)]
        node = BaseDriver(cli_args=arguments + (extra_args or []))
        nodes.append(node)
        return node, master

    yield create
    for node in nodes:
        if node._mcu_serial.is_open:
            node.destroy_node()
    for descriptor in descriptors:
        os.close(descriptor)
    rclpy.shutdown()


def receive(node, master, stream, imu_count, odom_count):
    """Collect real DDS publications from a supplied serial stream."""
    imu_messages = []
    odom_messages = []
    imu_sub = node.create_subscription(Imu, 'imu/data_raw', imu_messages.append, 10)
    odom_sub = node.create_subscription(Odometry, 'odom', odom_messages.append, 10)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if (node._imu_publisher.get_subscription_count()
                and node._odom_publisher.get_subscription_count()):
            break
    os.write(master, stream)
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if len(imu_messages) >= imu_count and len(odom_messages) >= odom_count:
            break
    # Drain remaining callbacks to detect any unintended publications.
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.02)
    node.destroy_subscription(imu_sub)
    node.destroy_subscription(odom_sub)
    assert len(imu_messages) == imu_count
    assert len(odom_messages) == odom_count
    return imu_messages, odom_messages


def test_mixed_frames_publish_si_units_without_extra_odometry(driver_factory):
    """Use distinct wheel and IMU encodings and leave IMU orientation absent."""
    node, master = driver_factory()
    imu, odom = receive(node, master, WHEEL_FRAME + IMU_FRAME, 1, 1)
    msg = imu[0]
    assert msg.header.frame_id == 'imu_link'
    assert msg.header.stamp.sec > 0
    assert msg.orientation_covariance[0] == -1.0
    gyro = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
    accel = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
    assert gyro == pytest.approx([math.radians(v / 16.4) for v in (-49, 10, 13)])
    assert accel == pytest.approx([v * 9.80665 / 16384 for v in (600, -248, 15908)])
    assert list(msg.angular_velocity_covariance) == [0.0] * 9
    assert odom[0].twist.twist.linear.x == pytest.approx(100 * 0.0585 * math.pi / 4096)
    assert odom[0].twist.twist.angular.z == 0.0


def test_imu_only_does_not_integrate_odometry(driver_factory):
    """Reject damaged wheel data and keep the odometry unchanged by IMU."""
    node, master = driver_factory()
    receive(node, master, WHEEL_FRAME[:-1] + b'\x00' + IMU_FRAME, 1, 0)
    assert (node._odom_x, node._odom_y, node._odom_theta) == (0.0, 0.0, 0.0)


def test_configured_sensitivity_bias_and_covariance(driver_factory):
    """Apply calibration parameters with ROS units and XYZ variances."""
    node, master = driver_factory([
        '-p', 'imu_frame_id:=test_imu',
        '-p', 'imu_gyro_lsb_per_dps:=131.0',
        '-p', 'imu_accel_lsb_per_g:=8192.0',
        '-p', 'imu_angular_velocity_bias:=[0.01, 0.02, 0.03]',
        '-p', 'imu_linear_acceleration_bias:=[0.1, 0.2, 0.3]',
        '-p', 'imu_angular_velocity_covariance_diagonal:=[0.001, 0.002, 0.003]',
        '-p', 'imu_linear_acceleration_covariance_diagonal:=[0.1, 0.2, 0.3]',
    ])
    imu, _ = receive(node, master, IMU_FRAME, 1, 0)
    msg = imu[0]
    assert msg.header.frame_id == 'test_imu'
    assert msg.angular_velocity.x == pytest.approx(math.radians(-49 / 131) - 0.01)
    assert msg.linear_acceleration.z == pytest.approx(15908 * 9.80665 / 8192 - 0.3)
    assert list(msg.angular_velocity_covariance) == [0.001, 0, 0, 0, 0.002, 0, 0, 0, 0.003]
    assert list(msg.linear_acceleration_covariance) == [0.1, 0, 0, 0, 0.2, 0, 0, 0, 0.3]


def test_shutdown_releases_serial_port(driver_factory):
    """Leave the pseudoterminal available after the ROS node shuts down."""
    node, _ = driver_factory()
    assert node._mcu_serial.is_open
    # Check actual cleanup once here; remove it from fixture-owned cleanup.
    node.destroy_node()
    assert not node._mcu_serial.is_open


def test_yaw_uses_elapsed_time_and_directional_calibration(driver_factory):
    from dianshu_base.base_driver import METER_PER_PULSE, TREAD
    node, _ = driver_factory(['-p', 'odom_angular_scale_ccw:=1.28', '-p', 'odom_angular_scale_cw:=1.22'])
    rate = 200 * METER_PER_PULSE / TREAD
    for dt in (0.08, 0.12, 0.21, 0.09):
        _, yaw = node.process_velocity(100, 100, dt)
    assert yaw == pytest.approx(rate * 1.28)
    assert node._odom_theta == pytest.approx(rate * 1.28 * 0.5)
    node.process_velocity(-100, -100, 0.3)
    assert node._odom_theta == pytest.approx(rate * (1.28 * 0.5 - 1.22 * 0.3))


def test_batched_wheel_frames_share_elapsed_interval(driver_factory):
    from unittest.mock import Mock, patch
    from dianshu_base.base_driver import METER_PER_PULSE, TREAD
    node, _ = driver_factory()
    stream = bytes.fromhex('ff 01 00 64 00 64 fe') * 3
    serial = Mock(in_waiting=len(stream))
    serial.read.return_value = stream
    node._last_wheel_time = 10.0
    with patch.object(node, '_mcu_serial', serial), patch('dianshu_base.base_driver.time.monotonic', return_value=10.24):
        node._serial_listen()
    assert node._odom_theta == pytest.approx(200 * METER_PER_PULSE / TREAD * 0.24)


def test_curved_motion_integrates_constant_twist_arc(driver_factory):
    from dianshu_base.base_driver import METER_PER_PULSE, TREAD
    node, _ = driver_factory()
    v = 50 * METER_PER_PULSE
    w = 100 * METER_PER_PULSE / TREAD
    node.process_velocity(0, 100, 0.4)
    assert node._odom_x == pytest.approx(v / w * math.sin(w * 0.4))
    assert node._odom_y == pytest.approx(v / w * (1 - math.cos(w * 0.4)))


def test_stale_command_sends_zero_wheel_speeds(driver_factory):
    from geometry_msgs.msg import Twist
    node, master = driver_factory()
    msg = Twist()
    msg.angular.z = 0.185302
    node.cmd_vel_callback(msg)
    node._last_command_time -= 1.0
    node._command_watchdog()
    received = os.read(master, 1024)
    assert len(received) == 14
    assert received[-7:] == bytes.fromhex('ff 01 00 00 00 00 fe')
    assert node._last_command_time is None


@pytest.mark.parametrize('target', [0.1, 23.4, 168.5, -168.5, 200.0])
def test_firmware_quantization_preserves_average_speed(target):
    from dianshu_base.base_driver import quantize_speed
    residual = 0.0
    output = []
    for _ in range(1000):
        value, residual = quantize_speed(target, residual, 50.0)
        assert value % 50 == 0
        assert value * target >= 0
        output.append(value)
    assert abs(sum(output) - target * len(output)) < 50.0 + 1e-6


def test_stop_clears_quantization_residue(driver_factory):
    from geometry_msgs.msg import Twist
    node, master = driver_factory(['-p', 'wheel_speed_quantum_pps:=50.0'])
    msg = Twist()
    msg.angular.z = 0.185302
    node.cmd_vel_callback(msg)
    assert any(node._wheel_quantization_error)
    node.cmd_vel_callback(Twist())
    assert node._wheel_quantization_error == [0.0, 0.0]
    assert node._target_wheel_pps == (0.0, 0.0)
    assert os.read(master, 1024)[-7:] == bytes.fromhex('ff 01 00 00 00 00 fe')
