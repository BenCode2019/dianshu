import serial
import rclpy
import math
import time
import quaternion


from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from rclpy.node import Node

from dianshu_base.protocol import (
    FrameParser, IMU_RAW_FRAME, WHEEL_SPEED_FRAME, decode_imu_raw,
    decode_wheel_speed,
)

"""
    定义PPS为脉冲数/s 作为上位机与下位机通信的标准速度单位
    MCU -> pps(pulse per second)-> Linux
    MCU <- pps(pulse per second) <- Linux
    下位机目前不会对反向的轮子进行处理,需要在上位机处理

    舵机位置:使用脉冲数直接控制,在下位机进行回中误差校正,即上位机发送02 00 02 00是物理效果上的中位
    MCU -> pulse -> Linux
"""

# 小车本体相关数据
MCU_SERAIL_BAUDRATE = 115200  # 下位机串口波特率
SERVO_PULSE = 4096  # 舵机脉冲数
WHEEL_DIAMETER = 0.0585  # 轮子直径
METER_PER_PULSE = WHEEL_DIAMETER * math.pi / SERVO_PULSE
TREAD = 0.1044  # Reference track width (m); angular calibration is configured separately.

ODOM_FRAME = "odom"
BASE_FRAME = "base_link"

MAX_ANGULAR_VELOCITY = 4
MAX_LINEAR_VELOCITY = 0.2


def quantize_speed(target, residual, quantum):
    """Return representable PPS with bounded cumulative magnitude error."""
    if quantum <= 0:
        return round(target), 0.0
    value = abs(target) + residual
    pulses = math.floor((value + 1e-9) / quantum) * quantum
    return int(math.copysign(pulses, target)), value - pulses


class BaseDriver(Node):
    def __init__(self, **kwargs):
        """
        初始化BaseDriver类的实例

        Args:
            None

        Returns:
            None
        """
        super().__init__("base_driver", **kwargs)
        self.get_logger().info("base_driver node start")

        self.declare_parameter("lower_mcu_serial_port", "/dev/ttyUSB0")
        self._mcu_serial_port = self.get_parameter("lower_mcu_serial_port").value
        self.get_logger().info("serial_port:{}".format(self._mcu_serial_port))

        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("imu_gyro_lsb_per_dps", 16.4)
        self.declare_parameter("imu_accel_lsb_per_g", 16384.0)
        self.declare_parameter("publish_odom_tf", True)
        self._imu_frame_id = self.get_parameter("imu_frame_id").value
        if not self._imu_frame_id or self._imu_frame_id.startswith("/"):
            raise ValueError("imu_frame_id must be nonempty and have no leading slash")
        self._gyro_lsb_per_dps = self.get_parameter("imu_gyro_lsb_per_dps").value
        self._accel_lsb_per_g = self.get_parameter("imu_accel_lsb_per_g").value
        for name, value in (
            ("imu_gyro_lsb_per_dps", self._gyro_lsb_per_dps),
            ("imu_accel_lsb_per_g", self._accel_lsb_per_g),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("{} must be finite and positive".format(name))
        self._gyro_bias = self._imu_vector_parameter("imu_angular_velocity_bias")
        self._accel_bias = self._imu_vector_parameter("imu_linear_acceleration_bias")
        self._gyro_covariance = self._imu_covariance_parameter(
            "imu_angular_velocity_covariance_diagonal"
        )
        self._accel_covariance = self._imu_covariance_parameter(
            "imu_linear_acceleration_covariance_diagonal"
        )
        self._publish_odom_tf = self.get_parameter("publish_odom_tf").value
        self._command_yaw_scale = []
        self._odom_yaw_scale = []
        for prefix, target in (("command", self._command_yaw_scale), ("odom", self._odom_yaw_scale)):
            for direction in ("ccw", "cw"):
                name = prefix + "_angular_scale_" + direction
                self.declare_parameter(name, 1.0)
                value = self.get_parameter(name).value
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(name + " must be finite and positive")
                target.append(value)
        self.declare_parameter("cmd_vel_timeout", 0.5)
        self._cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout").value)
        if not math.isfinite(self._cmd_vel_timeout) or self._cmd_vel_timeout <= 0:
            raise ValueError("cmd_vel_timeout must be finite and positive")
        self._last_command_time = None
        self._target_wheel_pps = (0.0, 0.0)
        self._wheel_quantization_error = [0.0, 0.0]
        self.declare_parameter("wheel_speed_quantum_pps", 0.0)
        self._wheel_speed_quantum = float(self.get_parameter("wheel_speed_quantum_pps").value)
        if not math.isfinite(self._wheel_speed_quantum) or self._wheel_speed_quantum < 0:
            raise ValueError("wheel_speed_quantum_pps must be finite and nonnegative")
        self._last_wheel_time = None
        self._frame_parser = FrameParser()
        # Poll without blocking the ROS executor; one reader handles both frames.
        self._mcu_serial = serial.Serial(
            self._mcu_serial_port, MCU_SERAIL_BAUDRATE, timeout=0, exclusive=True
        )
        self._cmd_vel_subscription = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )
        self._cmd_vel_subscription

        self._camera_pos_subscription = self.create_subscription(
            Int16MultiArray, "camera_pos", self.camera_pos_callback, 10
        )
        self._camera_pos_subscription

        self._odom_tf_broadcaster = TransformBroadcaster(self)
        self._odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self._imu_publisher = self.create_publisher(Imu, "imu/data_raw", 10)

        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_theta = 0.0
        self._debug_flag = False  # DEBUG

        self._serial_timer = self.create_timer(0.01, self._serial_listen)
        self._command_timer = self.create_timer(0.05, self._command_watchdog)

        self.get_logger().info("base_driver init success")

    def _imu_vector_parameter(self, name):
        """Read a finite three-axis IMU parameter in sensor XYZ order."""
        self.declare_parameter(name, [0.0, 0.0, 0.0])
        values = self.get_parameter(name).value
        if len(values) != 3 or not all(math.isfinite(v) for v in values):
            raise ValueError("{} must contain three finite numbers".format(name))
        return values

    def _imu_covariance_parameter(self, name):
        """Expand three nonnegative variances into a row-major covariance."""
        diagonal = self._imu_vector_parameter(name)
        if any(v < 0 for v in diagonal):
            raise ValueError("{} variances must be nonnegative".format(name))
        covariance = [0.0] * 9
        covariance[0], covariance[4], covariance[8] = diagonal
        return covariance

    def _serial_listen(self) -> None:
        """Read available bytes and dispatch complete wheel and IMU frames."""
        try:
            waiting = self._mcu_serial.in_waiting
            if not waiting:
                return
            data = self._mcu_serial.read(min(waiting, 4096))
        except (serial.SerialException, OSError) as error:
            self.get_logger().error("MCU serial read failed: {}".format(error))
            self._serial_timer.cancel()
            return
        frames = self._frame_parser.feed(data)
        wheel_count = sum(frame[1] == WHEEL_SPEED_FRAME for frame in frames)
        dt = 0.0
        if wheel_count:
            now = time.monotonic()
            if self._last_wheel_time is not None:
                elapsed = now - self._last_wheel_time
                # Firmware has no sample timestamps. Share one receive interval
                # across a batch; do not invent travel across a prolonged gap.
                if 0 < elapsed <= 0.5:
                    dt = elapsed / wheel_count
                else:
                    self.get_logger().warn("Wheel telemetry gap; skipped pose integration")
            self._last_wheel_time = now
        for frame in frames:
            if frame[1] == WHEEL_SPEED_FRAME:
                left_pps, right_pps = self.data_parse(frame)
                [linear_velocity, angular_velocity] = self.process_velocity(left_pps, right_pps, dt)
                if self._debug_flag:
                    self.get_logger().warn("raw_data :{}".format(frame.hex()))
                    self._debug_flag = False
                self.odom_publish(linear_velocity, angular_velocity)
            elif frame[1] == IMU_RAW_FRAME:
                self.imu_publish(frame)

    def imu_publish(self, frame: bytes) -> None:
        """Publish gyro and specific force in the MPU6050 sensor frame."""
        raw = decode_imu_raw(frame)
        gyro = [math.radians(v / self._gyro_lsb_per_dps) - bias
                for v, bias in zip(raw[:3], self._gyro_bias)]
        accel = [v * 9.80665 / self._accel_lsb_per_g - bias
                 for v, bias in zip(raw[3:], self._accel_bias)]
        msg = Imu()
        # The MCU protocol has no acquisition timestamp: use ROS receipt time.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._imu_frame_id
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0  # No orientation estimate on the wire.
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = accel
        msg.angular_velocity_covariance = self._gyro_covariance
        msg.linear_acceleration_covariance = self._accel_covariance
        self._imu_publisher.publish(msg)

    def destroy_node(self):
        """Stop serial polling and release the MCU port on shutdown."""
        self._serial_timer.cancel()
        self._command_timer.cancel()
        if self._mcu_serial.is_open:
            try:
                self.set_car_speed(0, 0)
            finally:
                self._mcu_serial.close()
        return super().destroy_node()

    def camera_pos_callback(self, msg: Int16MultiArray) -> None:
        try:
            x_position = msg.data[0]
            y_position = msg.data[1]
            self.set_camera_pos(x_position, y_position)
        except Exception:
            pass

    def set_camera_pos(self, x_position: int, y_position: int) -> None:
        """
        设置云台位置(暂定)

        Args:
            x_position (int): x轴云台位置
            y_position (int): y轴云台位置

        Returns:
            None
        """
        # 溢出判定 首位留给符号位

        x_position_h = int(x_position / 256)
        x_position_l = x_position % 256

        y_position_h = int(y_position / 256)
        y_position_l = y_position % 256

        data_frame_temp = [
            0xFF, 0x02, x_position_h, x_position_l, y_position_h, y_position_l, 0xFE
        ]
        data_frame = ""

        for element in data_frame_temp:
            data_frame = " ".join([data_frame, hex(element)[2:].rjust(2, "0")])
        # self.get_logger().info("data_frame:{}".format(data_frame)) #DEBUG
        self._mcu_serial.write(bytes.fromhex(data_frame))

    def cmd_vel_callback(self, msg: Twist) -> None:
        """
         cmd_vel topic的回调,将线速度和角速度转换为左右轮速度并下发

        Args:
            msg: geometry_msgs/Twist消息

        Returns:
            None
        """
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            self._stop_wheels()
            return
        yaw = msg.angular.z * self._command_yaw_scale[0 if msg.angular.z >= 0 else 1]
        left_speed = msg.linear.x - yaw * TREAD / 2
        right_speed = msg.linear.x + yaw * TREAD / 2

        left_pps = -left_speed / METER_PER_PULSE
        right_pps = right_speed / METER_PER_PULSE
        if abs(left_pps) >= 0x7FFF or abs(right_pps) >= 0x7FFF:
            self.get_logger().warn("Wheel command exceeds protocol range; stopping")
            self._stop_wheels()
            return
        previous = self._target_wheel_pps
        self._target_wheel_pps = (left_pps, right_pps)
        for i, value in enumerate(self._target_wheel_pps):
            if value == 0 or previous[i] * value < 0:
                self._wheel_quantization_error[i] = 0.0
        first_command = self._last_command_time is None
        self._last_command_time = time.monotonic()
        if left_pps == 0 and right_pps == 0:
            self._stop_wheels()
        elif first_command:
            self._send_target_speed()

    def _stop_wheels(self):
        self._target_wheel_pps = (0.0, 0.0)
        self._wheel_quantization_error = [0.0, 0.0]
        self._last_command_time = None
        self.set_car_speed(0, 0)

    def _send_target_speed(self):
        output = []
        for i, target in enumerate(self._target_wheel_pps):
            value, error = quantize_speed(target, self._wheel_quantization_error[i],
                                          self._wheel_speed_quantum)
            self._wheel_quantization_error[i] = error
            output.append(value)
        self.set_car_speed(*output)

    def _command_watchdog(self):
        if self._last_command_time is None:
            return
        if time.monotonic() - self._last_command_time > self._cmd_vel_timeout:
            self._stop_wheels()
        else:
            # Fixed 20 Hz pulse-density modulation compensates firmware speed
            # quantization without biasing the requested average wheel speed.
            self._send_target_speed()

    def set_car_speed(self, left_pps: int, right_pps: int) -> None:
        """
        设置车辆速度

        Args:
            left_pps (int): 左轮脉冲速度
            right_pps (int): 右轮脉冲速度

        Returns:
            None
        """
        # 溢出判定 首位留给符号位
        if abs(left_pps) >= 0x7FFF or abs(right_pps) >= 0x7FFF:
            self.get_logger().warn("速度过大")
            return
        if left_pps >= 0:
            left_h = int(left_pps / 256)
            left_l = left_pps % 256
        else:
            left_pps = -left_pps
            left_h = int(left_pps / 256)
            left_h += 0x80
            left_l = left_pps % 256
        if right_pps >= 0:
            right_h = int(right_pps / 256)
            right_l = right_pps % 256
        else:
            right_pps = -right_pps
            right_h = int(right_pps / 256)
            right_h += 0x80
            right_l = right_pps % 256
        data_frame_temp = [0xFF, 0x01, left_h, left_l, right_h, right_l, 0xFE]
        data_frame = ""
        for element in data_frame_temp:
            data_frame = " ".join([data_frame, hex(element)[2:].rjust(2, "0")])
        self.get_logger().debug(f"发送速度指令:{data_frame}")
        self._mcu_serial.write(bytes.fromhex(data_frame))

    def odom_publish(self, linear_velocity: int, angular_velocity: int) -> None:
        """
        发布里程计topic和tf

        Args:
            linear_velocity (int): 线速度
            angular_velocity (int): 角速度

        Returns:
            None
        """
        self.get_logger().debug(
            "now_speed({}):{}|{}".format(time.perf_counter(), linear_velocity, angular_velocity)
        )

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = ODOM_FRAME
        odom_msg.child_frame_id = BASE_FRAME
        odom_msg.pose.pose.position.x = self._odom_x
        odom_msg.pose.pose.position.y = self._odom_y
        odom_msg.pose.pose.position.z = 0.0  # NOTE msg赋值对类型强要求

        quat = quaternion.from_euler_angles(0, 0, self._odom_theta)

        odom_msg.pose.pose.orientation.x = quat.x
        odom_msg.pose.pose.orientation.y = quat.y
        odom_msg.pose.pose.orientation.z = quat.z
        odom_msg.pose.pose.orientation.w = quat.w

        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = angular_velocity

        self._odom_publisher.publish(odom_msg)

        tf_odom = TransformStamped()
        tf_odom.header.stamp = self.get_clock().now().to_msg()
        tf_odom.header.frame_id = ODOM_FRAME
        tf_odom.child_frame_id = BASE_FRAME

        tf_odom.transform.translation.x = self._odom_x
        tf_odom.transform.translation.y = self._odom_y
        self.get_logger().debug("{} {}".format(self._odom_x, self._odom_y))
        tf_odom.transform.translation.z = 0.0
        tf_odom.transform.rotation.x = quat.x
        tf_odom.transform.rotation.y = quat.y
        tf_odom.transform.rotation.z = quat.z
        tf_odom.transform.rotation.w = quat.w

        if self._publish_odom_tf:
            self._odom_tf_broadcaster.sendTransform(tf_odom)

    def data_parse(self, data: bytes) -> [int, int]:
        """
        解析数据字节流，提取左右脉冲速度。

        Args:
            data (bytes): 待解析的数据字节流

        Returns:
            list[int, int]: 包含左右脉冲速度的列表
        """
        self.get_logger().debug("raw_data:{}".format(data.hex()))
        return decode_wheel_speed(data)

    def process_velocity(self, left_pps: int, right_pps: int, dt: float) -> [float, float]:
        """
        将左右轮的脉冲速度转换为线速度和角速度

        Args:
            left_pps (int): 左轮脉冲速度
            right_pps (int): 右轮脉冲速度

        Returns:
            list[float, float]: 包含线速度和角速度的列表
        """
        left_speed = -left_pps * METER_PER_PULSE  # NOTE 左轮反向
        right_speed = right_pps * METER_PER_PULSE

        linear_velocity = (left_speed + right_speed) / 2
        angular_velocity = (right_speed - left_speed) / TREAD

        angular_velocity *= self._odom_yaw_scale[0 if angular_velocity >= 0 else 1]
        dtheta = angular_velocity * dt
        # Exact constant-twist planar integration, continuous at zero yaw rate.
        distance = linear_velocity * dt
        if abs(dtheta) > 1e-9:
            distance *= math.sin(dtheta / 2) / (dtheta / 2)
        heading = self._odom_theta + dtheta / 2
        self._odom_x += distance * math.cos(heading)
        self._odom_y += distance * math.sin(heading)
        self._odom_theta = math.atan2(math.sin(self._odom_theta + dtheta),
                                     math.cos(self._odom_theta + dtheta))

        return [float(linear_velocity), float(angular_velocity)]  # 避免0为int


def main(args=None):
    """
    主函数,用于初始化ROS节点并启动BaseDriver

    Args:
        args (list): 命令行参数列表,默认为None

    Returns:
        None
    """
    rclpy.init(args=args)

    newbee_car_driver_node = BaseDriver()

    try:
        rclpy.spin(newbee_car_driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        newbee_car_driver_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
