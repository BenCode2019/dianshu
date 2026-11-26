from pathlib import Path
import shutil,datetime
root=Path('src/sheng-xiao-robot_-dian-shu_-ros/dianshu_base');backup=Path('calibration-work/backup-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
for rel in ['dianshu_base/base_driver.py','launch/base_launch.py','config/imu.yaml','README.md']:
 dest=backup/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(root/rel,dest)
p=root/'dianshu_base/base_driver.py';s=p.read_text()
s=s.replace('TREAD = 0.1044  # 有效轮距(m)，按恒定角速度 0.185302 rad/s 转一圈 40 s 初步标定，需复测','TREAD = 0.1044  # Reference track width (m); angular calibration is configured separately.')
s=s.replace('        self._frame_parser = FrameParser()','''        self._command_yaw_scale = []
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
        self._last_wheel_time = None
        self._frame_parser = FrameParser()''')
s=s.replace('        self._serial_timer = self.create_timer(0.01, self._serial_listen)','''        self._serial_timer = self.create_timer(0.01, self._serial_listen)
        self._command_timer = self.create_timer(0.05, self._command_watchdog)''')
s=s.replace('        for frame in self._frame_parser.feed(data):','''        frames = self._frame_parser.feed(data)
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
        for frame in frames:''')
s=s.replace('self.process_velocity(left_pps, right_pps)','self.process_velocity(left_pps, right_pps, dt)')
s=s.replace('        self._serial_timer.cancel()\n        self._mcu_serial.close()','''        self._serial_timer.cancel()
        self._command_timer.cancel()
        if self._mcu_serial.is_open:
            try:
                self.set_car_speed(0, 0)
            finally:
                self._mcu_serial.close()''')
s=s.replace('        left_speed = msg.linear.x - msg.angular.z * TREAD / 2\n        right_speed = msg.linear.x + msg.angular.z * TREAD / 2','''        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            self.set_car_speed(0, 0)
            self._last_command_time = None
            return
        yaw = msg.angular.z * self._command_yaw_scale[0 if msg.angular.z >= 0 else 1]
        left_speed = msg.linear.x - yaw * TREAD / 2
        right_speed = msg.linear.x + yaw * TREAD / 2''')
s=s.replace('        )  # NOTE 左轮需反向 # TODO 这个事情是否应该在下位机进行处理?','''        )  # Left motor and encoder signs are reversed on this chassis.
        self._last_command_time = time.monotonic()

    def _command_watchdog(self):
        if (self._last_command_time is not None
                and time.monotonic() - self._last_command_time > self._cmd_vel_timeout):
            self.set_car_speed(0, 0)
            self._last_command_time = None''')
s=s.replace('def process_velocity(self, left_pps: int, right_pps: int)', 'def process_velocity(self, left_pps: int, right_pps: int, dt: float)')
a=s.index('        self._odom_x += linear_velocity * math.cos(self._odom_theta) * 0.1')
b=s.index('        return [float(linear_velocity), float(angular_velocity)]',a)
s=s[:a]+'''        angular_velocity *= self._odom_yaw_scale[0 if angular_velocity >= 0 else 1]
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

'''+s[b:]
compile(s,str(p),'exec');p.write_text(s)
p=root/'launch/base_launch.py';s=p.read_text();anchor='        DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),';s=s.replace(anchor,'''        DeclareLaunchArgument(
            "drive_params_file",
            default_value=PathJoinSubstitution([FindPackageShare("dianshu_base"), "config", "drive.yaml"]),
            description="Angular command and wheel odometry calibration",
        ),
'''+anchor);s=s.replace('            LaunchConfiguration("imu_params_file"),','            LaunchConfiguration("imu_params_file"),\n            LaunchConfiguration("drive_params_file"),');compile(s,str(p),'exec');p.write_text(s)
(root/'config/drive.yaml').write_text('''/**:
  ros__parameters:
    # Independent command and feedback gains, fitted against LiDAR scan matching.
    # Direction is ROS yaw: ccw positive, cw negative.
    command_angular_scale_ccw: 0.90
    command_angular_scale_cw: 0.92
    odom_angular_scale_ccw: 1.28
    odom_angular_scale_cw: 1.22
    cmd_vel_timeout: 0.5
''')
print('Backup:',backup)
