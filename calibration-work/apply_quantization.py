from pathlib import Path
p=Path('src/sheng-xiao-robot_-dian-shu_-ros/dianshu_base/dianshu_base/base_driver.py');s=p.read_text();Path('calibration-work/base_driver-before-quantization.py').write_text(s)
s=s.replace('        self._last_command_time = None\n        self._last_wheel_time = None','''        self._last_command_time = None
        self._target_wheel_pps = (0.0, 0.0)
        self._wheel_quantization_error = [0.0, 0.0]
        self.declare_parameter("wheel_speed_quantum_pps", 0.0)
        self._wheel_speed_quantum = float(self.get_parameter("wheel_speed_quantum_pps").value)
        if not math.isfinite(self._wheel_speed_quantum) or self._wheel_speed_quantum < 0:
            raise ValueError("wheel_speed_quantum_pps must be finite and nonnegative")
        self._last_wheel_time = None''')
a=s.index('        left_speed = round(left_speed /');b=s.index('    def set_car_speed(',a)
s=s[:a]+'''        left_pps = -left_speed / METER_PER_PULSE
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

'''+s[b:]
s=s.replace('            self.set_car_speed(0, 0)\n            self._last_command_time = None\n            return','            self._stop_wheels()\n            return',1)
anchor='class BaseDriver(Node):'
s=s.replace(anchor,'''def quantize_speed(target, residual, quantum):
    """Return representable PPS with bounded cumulative magnitude error."""
    if quantum <= 0:
        return round(target), 0.0
    value = abs(target) + residual
    pulses = math.floor((value + 1e-9) / quantum) * quantum
    return int(math.copysign(pulses, target)), value - pulses


'''+anchor)
compile(s,str(p),'exec');p.write_text(s)
p=Path('src/sheng-xiao-robot_-dian-shu_-ros/dianshu_base/config/drive.yaml');s=p.read_text().replace('command_angular_scale_ccw: 0.90','command_angular_scale_ccw: 0.78125').replace('command_angular_scale_cw: 0.92','command_angular_scale_cw: 0.819672131147541');s+='    # Measured firmware plateaus in a 185/195/200/205/215 PPS sweep.\n    wheel_speed_quantum_pps: 50.0\n';p.write_text(s)
