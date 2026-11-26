"""Decode the B20 wheel speed and MPU6050 raw telemetry frames."""

import struct


WHEEL_SPEED_FRAME = 0x01
IMU_RAW_FRAME = 0x04
FRAME_LENGTHS = {WHEEL_SPEED_FRAME: 7, IMU_RAW_FRAME: 15}


class FrameParser:
    """Buffer partial frames and recover frame alignment after invalid bytes."""

    def __init__(self):
        """Start with an empty receive buffer."""
        self._buffer = bytearray()

    def feed(self, data):
        """Return complete known frames, preserving any incomplete final frame."""
        self._buffer.extend(data)
        frames = []
        while self._buffer:
            start = self._buffer.find(b'\xff')
            if start < 0:
                self._buffer.clear()
                break
            del self._buffer[:start]
            if len(self._buffer) < 2:
                break
            length = FRAME_LENGTHS.get(self._buffer[1])
            if length is None:
                del self._buffer[0]
                continue
            if len(self._buffer) < length:
                break
            # FF/FE may occur inside either payload: only check the fixed tail.
            if self._buffer[length - 1] != 0xFE:
                del self._buffer[0]
                continue
            frames.append(bytes(self._buffer[:length]))
            del self._buffer[:length]
        return frames


def _check_frame(frame, frame_type):
    if (len(frame) != FRAME_LENGTHS[frame_type]
            or frame[0] != 0xFF or frame[1] != frame_type or frame[-1] != 0xFE):
        raise ValueError('Invalid B20 frame: {}'.format(frame.hex()))


def decode_wheel_speed(frame):
    """Decode the two sign-magnitude wheel speeds, in pulses per second."""
    _check_frame(frame, WHEEL_SPEED_FRAME)
    values = struct.unpack('>2H', frame[2:-1])
    return tuple(-(v & 0x7FFF) if v & 0x8000 else v for v in values)


def decode_imu_raw(frame):
    """Return gyro XYZ then accelerometer XYZ as signed raw sensor counts."""
    _check_frame(frame, IMU_RAW_FRAME)
    return struct.unpack('>6h', frame[2:-1])
