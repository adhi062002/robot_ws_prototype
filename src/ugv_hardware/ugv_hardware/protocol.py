"""Teensy <-> Jetson wire protocol (Jetson side).

Mirror of firmware/teensy41_ugv/ugv_protocol.h -- keep both in sync;
test/test_protocol.py compiles the C++ header and cross-checks every message.

Frame (little-endian):  A5 5A | type | len | seq | payload | crc16
crc16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type..payload.

Pure Python, no ROS imports, so it can be used and tested anywhere.
"""

import struct
from dataclasses import dataclass
from typing import List, Optional, Tuple

SYNC = b"\xA5\x5A"
MAX_PAYLOAD = 120
FRAME_OVERHEAD = 7
N_WHEELS = 4  # order: 0=FL 1=FR 2=RL 3=RR

MSG_TELEMETRY = 0x01
MSG_CONFIG_REPORT = 0x02
MSG_CMD_WHEELS = 0x10
MSG_CONFIG_SET = 0x11
MSG_CLEAR_FAULT = 0x12
MSG_REQUEST_CONFIG = 0x13
MSG_CMD_SERVOS = 0x14

ST_ESTOP = 1 << 0
ST_CMD_WATCHDOG = 1 << 1
ST_IMU_STALE = 1 << 2
ST_ENC_FAULT = 1 << 3
ST_MOTOR_FAULT = 1 << 4
ST_CONFIG_VALID = 1 << 5
ST_LOW_BATTERY = 1 << 6
ST_ARMED = 1 << 7
ST_OPEN_LOOP = 1 << 8

STATUS_NAMES = {
    ST_ESTOP: "ESTOP", ST_CMD_WATCHDOG: "CMD_WATCHDOG", ST_IMU_STALE: "IMU_STALE",
    ST_ENC_FAULT: "ENC_FAULT", ST_MOTOR_FAULT: "MOTOR_FAULT",
    ST_CONFIG_VALID: "CONFIG_VALID", ST_LOW_BATTERY: "LOW_BATTERY", ST_ARMED: "ARMED",
    ST_OPEN_LOOP: "OPEN_LOOP",
}
# Conditions that mean "do not trust / do not expect motion".
FAULT_MASK = ST_ESTOP | ST_ENC_FAULT | ST_MOTOR_FAULT

CMD_ENABLE = 1 << 0

TELEMETRY_FMT = "<I4i4f3f3fIHHHB"
CMD_WHEELS_FMT = "<ffB"
CONFIG_FMT = "<5fHBB"
CMD_SERVOS_FMT = "<hh"
assert struct.calcsize(TELEMETRY_FMT) == 71
assert struct.calcsize(CMD_WHEELS_FMT) == 9
assert struct.calcsize(CONFIG_FMT) == 24
assert struct.calcsize(CMD_SERVOS_FMT) == 4


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(msg_type: int, payload: bytes = b"", seq: int = 0) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload {len(payload)} > {MAX_PAYLOAD}")
    body = bytes([msg_type, len(payload), seq & 0xFF]) + payload
    return SYNC + body + struct.pack("<H", crc16_ccitt(body))


@dataclass
class Frame:
    msg_type: int
    seq: int
    payload: bytes


class FrameParser:
    """Incremental parser. feed() accepts arbitrary chunks (even single bytes)
    and returns every complete, CRC-valid frame. After a CRC error it rescans
    from just past the sync pair, so a good frame hiding inside is recovered."""

    def __init__(self):
        self._buf = bytearray()
        self.crc_errors = 0
        self.bad_len = 0
        self.bytes_discarded = 0

    def feed(self, data: bytes) -> List[Frame]:
        self._buf += data
        frames: List[Frame] = []
        while True:
            i = self._buf.find(SYNC)
            if i < 0:
                keep = 1 if self._buf.endswith(SYNC[:1]) else 0  # maybe half a sync
                drop = len(self._buf) - keep
                self.bytes_discarded += drop
                del self._buf[:drop]
                break
            if i > 0:
                self.bytes_discarded += i
                del self._buf[:i]
            if len(self._buf) < 5:
                break
            length = self._buf[3]
            if length > MAX_PAYLOAD:
                self.bad_len += 1
                del self._buf[:2]
                continue
            total = length + FRAME_OVERHEAD
            if len(self._buf) < total:
                break
            raw = bytes(self._buf[:total])
            if crc16_ccitt(raw[2:-2]) != struct.unpack("<H", raw[-2:])[0]:
                self.crc_errors += 1
                del self._buf[:2]
                continue
            frames.append(Frame(raw[2], raw[4], raw[5:-2]))
            del self._buf[:total]
        return frames


@dataclass
class Telemetry:
    t_us: int
    enc_ticks: Tuple[int, int, int, int]
    wheel_vel: Tuple[float, float, float, float]   # rad/s
    gyro: Tuple[float, float, float]               # rad/s
    accel: Tuple[float, float, float]              # m/s^2
    imu_t_us: int
    status: int
    cmd_age_ms: int
    batt_mv: int
    enc_ok_mask: int

    @classmethod
    def from_payload(cls, payload: bytes) -> "Telemetry":
        if len(payload) != struct.calcsize(TELEMETRY_FMT):
            raise ValueError(f"telemetry payload is {len(payload)} bytes, expected 71")
        v = struct.unpack(TELEMETRY_FMT, payload)
        return cls(v[0], tuple(v[1:5]), tuple(v[5:9]), tuple(v[9:12]), tuple(v[12:15]),
                   v[15], v[16], v[17], v[18], v[19])

    def to_payload(self) -> bytes:
        return struct.pack(TELEMETRY_FMT, self.t_us, *self.enc_ticks, *self.wheel_vel,
                           *self.gyro, *self.accel, self.imu_t_us, self.status,
                           self.cmd_age_ms, self.batt_mv, self.enc_ok_mask)

    def status_names(self) -> List[str]:
        return [n for bit, n in STATUS_NAMES.items() if self.status & bit]

    @property
    def armed(self) -> bool:
        return bool(self.status & ST_ARMED)

    @property
    def has_fault(self) -> bool:
        return bool(self.status & FAULT_MASK)


@dataclass
class Config:
    ticks_per_wheel_rev: float
    kp: float
    ki: float
    kd: float
    max_wheel_rad_s: float
    watchdog_ms: int = 250
    enc_invert_mask: int = 0
    motor_invert_mask: int = 0

    def to_payload(self) -> bytes:
        return struct.pack(CONFIG_FMT, self.ticks_per_wheel_rev, self.kp, self.ki, self.kd,
                           self.max_wheel_rad_s, self.watchdog_ms, self.enc_invert_mask,
                           self.motor_invert_mask)

    @classmethod
    def from_payload(cls, payload: bytes) -> "Config":
        if len(payload) != struct.calcsize(CONFIG_FMT):
            raise ValueError(f"config payload is {len(payload)} bytes, expected 24")
        return cls(*struct.unpack(CONFIG_FMT, payload))


def pack_cmd_wheels(left_rad_s: float, right_rad_s: float, enable: bool = True) -> bytes:
    return struct.pack(CMD_WHEELS_FMT, left_rad_s, right_rad_s, CMD_ENABLE if enable else 0)


def unpack_cmd_wheels(payload: bytes) -> Tuple[float, float, bool]:
    left, right, flags = struct.unpack(CMD_WHEELS_FMT, payload)
    return left, right, bool(flags & CMD_ENABLE)


def pack_cmd_servos(servo1_deg: int, servo2_deg: int) -> bytes:
    return struct.pack(CMD_SERVOS_FMT, int(servo1_deg), int(servo2_deg))


def unpack_cmd_servos(payload: bytes) -> Tuple[int, int]:
    return struct.unpack(CMD_SERVOS_FMT, payload)


class MicrosUnwrapper:
    """Teensy micros() is a u32 that wraps every ~71.6 min. Returns a
    monotonic 64-bit microsecond count. Call with samples in arrival order."""

    def __init__(self):
        self._last: Optional[int] = None
        self._offset = 0

    def unwrap(self, t_us: int) -> int:
        if self._last is not None and t_us < self._last and (self._last - t_us) > 2**31:
            self._offset += 2**32
        self._last = t_us
        return t_us + self._offset


class SeqTracker:
    """Counts frames lost on the link from the u8 sequence number."""

    def __init__(self):
        self._last: Optional[int] = None
        self.received = 0
        self.dropped = 0

    def update(self, seq: int) -> int:
        """Returns how many frames were skipped before this one."""
        self.received += 1
        skipped = 0
        if self._last is not None:
            skipped = (seq - self._last - 1) & 0xFF
            if skipped > 128:      # looks like a reorder/restart, not a loss
                skipped = 0
            self.dropped += skipped
        self._last = seq
        return skipped
