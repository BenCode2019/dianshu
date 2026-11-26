from pathlib import Path
p=Path('src/sheng-xiao-robot_-dian-shu_-ros/dianshu_base/dianshu_base/base_driver.py');s=p.read_text();Path('calibration-work/base_driver-before-imu-yaw.py').write_text(s)
s=s.replace('import time\n','import time\nfrom collections import deque\n',1)
s=s.replace('        self._last_wheel_time = None','''        self.declare_parameter("odom_imu_weight", 0.0)
        self._odom_imu_weight = float(self.get_parameter("odom_imu_weight").value)
        if not 0.0 <= self._odom_imu_weight <= 1.0:
            raise ValueError("odom_imu_weight must be in [0, 1]")
        self._gyro_bias_samples = deque(maxlen=50)
        self._gyro_auto_bias = 0.0
        self._latest_imu_yaw = 0.0
        self._latest_imu_time = None
        self._wheels_stationary = True
        self._imu_fallback_warned = False
        self._last_wheel_time = None''',1)
s=s.replace('        msg = Imu()','''        now = time.monotonic()
        # Only learn residual Z bias while both wheels and command are stopped.
        if (self._wheels_stationary and not any(self._target_wheel_pps)
                and abs(gyro[2]) < 0.05):
            self._gyro_bias_samples.append(gyro[2])
            if len(self._gyro_bias_samples) >= 20:
                self._gyro_auto_bias = sum(self._gyro_bias_samples) / len(self._gyro_bias_samples)
        self._latest_imu_yaw = gyro[2] - self._gyro_auto_bias
        self._latest_imu_time = now
        msg = Imu()''',1)
anchor='        angular_velocity *= self._odom_yaw_scale[0 if angular_velocity >= 0 else 1]'
s=s.replace(anchor,anchor+'''
        self._wheels_stationary = left_pps == 0 and right_pps == 0
        if self._odom_imu_weight:
            ready = (self._latest_imu_time is not None
                     and time.monotonic() - self._latest_imu_time <= 0.25
                     and len(self._gyro_bias_samples) >= 20)
            if ready:
                gyro_yaw = self._latest_imu_yaw
                if (self._wheels_stationary and not any(self._target_wheel_pps)
                        and abs(gyro_yaw) < 0.015):
                    gyro_yaw = 0.0
                angular_velocity = ((1 - self._odom_imu_weight) * angular_velocity
                                    + self._odom_imu_weight * gyro_yaw)
                self._imu_fallback_warned = False
            elif not self._imu_fallback_warned:
                self.get_logger().warn("IMU warming up or stale; using calibrated wheel yaw")
                self._imu_fallback_warned = True
''')
s=s.replace('    except KeyboardInterrupt:\n        pass','    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):\n        pass')
compile(s,str(p),'exec');p.write_text(s)
p=Path('src/sheng-xiao-robot_-dian-shu_-ros/dianshu_base/config/drive.yaml');s=p.read_text();s+='    # Wheels provide translation, zero-bias-calibrated IMU supplies yaw.\n    # Keep stationary for 3 seconds after launch; stale IMU falls back to wheel yaw.\n    odom_imu_weight: 1.0\n';p.write_text(s)
