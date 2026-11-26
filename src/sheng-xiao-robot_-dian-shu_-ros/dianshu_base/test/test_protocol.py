"""Exercise mixed, fragmented and damaged B20 telemetry streams."""

import struct

import pytest

from dianshu_base.protocol import FrameParser, decode_imu_raw, decode_wheel_speed


WHEEL_FRAME = bytes.fromhex('ff 01 80 64 00 c8 fe')
# Captured on /dev/ttyUSB0; FF is also present in the gyro/accel payload.
IMU_FRAME = bytes.fromhex('ff 04 ff cf 00 0a 00 0d 02 58 ff 08 3e 24 fe')


@pytest.mark.parametrize('split', range(len(WHEEL_FRAME + IMU_FRAME) + 1))
def test_all_stream_split_positions(split):
    """Preserve both frames regardless of the serial read boundary."""
    stream = WHEEL_FRAME + IMU_FRAME
    parser = FrameParser()
    assert parser.feed(stream[:split]) + parser.feed(stream[split:]) == [
        WHEEL_FRAME, IMU_FRAME,
    ]


def test_payload_contains_headers_and_tails():
    """Treat embedded FF 01, FF 04 and FE bytes as sensor counts."""
    payload = struct.pack('>6h', -255, -252, -2, -32768, 32767, 16384)
    frame = b'\xff\x04' + payload + b'\xfe'
    parser = FrameParser()
    frames = []
    for value in frame + WHEEL_FRAME:
        frames.extend(parser.feed(bytes([value])))
    assert frames == [frame, WHEEL_FRAME]
    assert decode_imu_raw(frame) == (-255, -252, -2, -32768, 32767, 16384)


def test_recover_after_noise_unknown_type_and_bad_tail():
    """Resynchronize without emitting a zero-speed frame for invalid data."""
    parser = FrameParser()
    stream = b'\x12\xfe\xff\x77\x00' + WHEEL_FRAME[:-1] + b'\x00'
    stream += IMU_FRAME + WHEEL_FRAME
    assert parser.feed(stream) == [IMU_FRAME, WHEEL_FRAME]


def test_start_in_middle_of_frame():
    """Recover after opening the serial port during an existing IMU frame."""
    parser = FrameParser()
    assert parser.feed(IMU_FRAME[5:] + WHEEL_FRAME + IMU_FRAME) == [
        WHEEL_FRAME, IMU_FRAME,
    ]


def test_wire_encodings():
    """Distinguish wheel sign-magnitude from IMU two's complement."""
    assert decode_wheel_speed(WHEEL_FRAME) == (-100, 200)
    assert decode_imu_raw(IMU_FRAME) == (-49, 10, 13, 600, -248, 15908)
    assert decode_wheel_speed(bytes.fromhex('ff 01 80 00 7f ff fe')) == (0, 32767)


@pytest.mark.parametrize('decoder, frame', [
    (decode_wheel_speed, b''),
    (decode_wheel_speed, IMU_FRAME),
    (decode_wheel_speed, WHEEL_FRAME[:-1] + b'\x00'),
    (decode_imu_raw, b''),
    (decode_imu_raw, WHEEL_FRAME),
    (decode_imu_raw, IMU_FRAME[:-1]),
])
def test_reject_invalid_frame(decoder, frame):
    """Reject missing bytes, wrong frame types and invalid tails."""
    with pytest.raises(ValueError):
        decoder(frame)
