"""Protocol tests, including a cross-check against the real C++ header.

Run:  cd src/ugv_hardware && python -m pytest test/test_protocol.py -q
(the C++ cross-check needs g++ and is skipped if it is missing).
"""
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ugv_hardware"))
import protocol as P  # noqa: E402

CPP = ROOT / "firmware" / "test" / "protocol_host_test.cpp"


def test_crc_check_value():
    assert P.crc16_ccitt(b"123456789") == 0x29B1   # CRC-16/CCITT-FALSE standard check


def test_telemetry_roundtrip_and_status_helpers():
    t = P.Telemetry(123456, (1, -2, 3, -4), (1.5, -2.5, 0.0, 0.25), (0.5, 0.25, -1.0),
                    (0.0, 9.5, -0.5), 123000, P.ST_ARMED | P.ST_CONFIG_VALID | P.ST_ESTOP,
                    17, 11100, 0x0F)
    frames = P.FrameParser().feed(P.encode_frame(P.MSG_TELEMETRY, t.to_payload(), seq=3))
    assert len(frames) == 1 and frames[0].seq == 3
    back = P.Telemetry.from_payload(frames[0].payload)
    assert back == t
    assert back.armed and back.has_fault
    assert sorted(back.status_names()) == ["ARMED", "CONFIG_VALID", "ESTOP"]


def test_cmd_and_config_roundtrip():
    f = P.FrameParser().feed(P.encode_frame(P.MSG_CMD_WHEELS, P.pack_cmd_wheels(1.5, -2.0), 9))[0]
    assert P.unpack_cmd_wheels(f.payload) == (1.5, -2.0, True)
    cfg = P.Config(4096 * 2.5, 0.05, 0.4, 0.0, 20.0, 250, 0b0101, 0b1010)
    f = P.FrameParser().feed(P.encode_frame(P.MSG_CONFIG_SET, cfg.to_payload()))[0]
    back = P.Config.from_payload(f.payload)
    for name in ("ticks_per_wheel_rev", "kp", "ki", "kd", "max_wheel_rad_s"):
        assert getattr(back, name) == pytest.approx(getattr(cfg, name), rel=1e-6)  # float32
    assert (back.watchdog_ms, back.enc_invert_mask, back.motor_invert_mask) == (250, 5, 10)


def test_servo_command_and_open_loop_flag():
    f = P.FrameParser().feed(P.encode_frame(P.MSG_CMD_SERVOS, P.pack_cmd_servos(45, -10), 6))[0]
    assert f.msg_type == P.MSG_CMD_SERVOS and P.unpack_cmd_servos(f.payload) == (45, -10)
    t = P.Telemetry(1, (0,) * 4, (0.0,) * 4, (0.0,) * 3, (0.0,) * 3, 1,
                    P.ST_OPEN_LOOP | P.ST_CONFIG_VALID, 0, 0, 0)
    assert "OPEN_LOOP" in P.Telemetry.from_payload(t.to_payload()).status_names()
    assert not t.has_fault           # open loop is a mode, not a fault


def test_parser_handles_byte_at_a_time_garbage_and_back_to_back_frames():
    a = P.encode_frame(P.MSG_CMD_WHEELS, P.pack_cmd_wheels(1.0, 1.0), 1)
    b = P.encode_frame(P.MSG_CLEAR_FAULT, b"", 2)
    stream = b"\x00\xff\xa5\x13\x37" + a + b"\xa5" + b + a
    p = P.FrameParser()
    got = []
    for byte in stream:               # worst case: one byte per read
        got += p.feed(bytes([byte]))
    assert [(f.msg_type, f.seq) for f in got] == [
        (P.MSG_CMD_WHEELS, 1), (P.MSG_CLEAR_FAULT, 2), (P.MSG_CMD_WHEELS, 1)]
    assert p.crc_errors == 0 and p.bytes_discarded >= 5


def test_corrupted_frame_is_counted_and_next_frame_still_arrives():
    good = P.encode_frame(P.MSG_CMD_WHEELS, P.pack_cmd_wheels(2.0, 2.0), 5)
    bad = bytearray(good)
    bad[6] ^= 0xFF                    # flip a payload byte
    p = P.FrameParser()
    got = p.feed(bytes(bad) + good)
    assert [f.seq for f in got] == [5] and p.crc_errors == 1


def test_oversize_length_is_rejected_and_parser_recovers():
    p = P.FrameParser()
    got = p.feed(b"\xA5\x5A\x01\xFF\x00" + P.encode_frame(P.MSG_CLEAR_FAULT, b"", 4))
    assert p.bad_len == 1 and [f.seq for f in got] == [4]
    with pytest.raises(ValueError):
        P.encode_frame(1, b"x" * 200)


def test_truncated_frame_waits_for_rest():
    frame = P.encode_frame(P.MSG_CMD_WHEELS, P.pack_cmd_wheels(1.0, 2.0), 8)
    p = P.FrameParser()
    assert p.feed(frame[:6]) == []
    assert [f.seq for f in p.feed(frame[6:])] == [8]


def test_micros_unwrapper_survives_u32_wrap():
    u = P.MicrosUnwrapper()
    seq = [4_000_000_000, 4_290_000_000, 5_000, 1_000_000]
    out = [u.unwrap(x) for x in seq]
    assert out == [4_000_000_000, 4_290_000_000, 2**32 + 5_000, 2**32 + 1_000_000]
    assert all(b > a for a, b in zip(out, out[1:]))


def test_seq_tracker_counts_drops_across_wrap():
    s = P.SeqTracker()
    for q in (254, 255, 0, 1):
        assert s.update(q) == 0
    assert s.update(5) == 3 and s.dropped == 3 and s.received == 5


# ---------------------------- C++ cross-check ----------------------------

@pytest.fixture(scope="module")
def host_bin(tmp_path_factory):
    if shutil.which("g++") is None:
        pytest.skip("g++ not available")
    out = tmp_path_factory.mktemp("cpp") / "protocol_host_test"
    r = subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", str(CPP),
                        "-o", str(out)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return str(out)


def run(host_bin, *args):
    return subprocess.run([host_bin, *args], capture_output=True, text=True,
                          check=True).stdout.strip()


def test_cpp_crc_matches_python(host_bin):
    assert run(host_bin, "crc") == "29B1"


def test_python_decodes_cpp_telemetry_frame(host_bin):
    hexframe = run(host_bin, "emit_telemetry")
    frames = P.FrameParser().feed(bytes.fromhex(hexframe))
    assert len(frames) == 1 and frames[0].msg_type == P.MSG_TELEMETRY and frames[0].seq == 7
    t = P.Telemetry.from_payload(frames[0].payload)
    assert t.t_us == 4_000_000_000 and t.imu_t_us == 3_999_990_000
    assert t.enc_ticks == (100, -200, 300, -2147483647)
    assert t.wheel_vel == (1.5, -2.25, 0.5, 0.0)
    assert t.gyro == (0.25, -0.5, 1.0) and t.accel == (0.0, 9.75, -1.5)
    assert t.status == P.ST_CONFIG_VALID | P.ST_ARMED
    assert (t.cmd_age_ms, t.batt_mv, t.enc_ok_mask) == (42, 11800, 0x0F)


def test_cpp_decodes_python_cmd_and_config_frames(host_bin):
    cmd = P.encode_frame(P.MSG_CMD_WHEELS, P.pack_cmd_wheels(1.5, -0.75), seq=200)
    assert run(host_bin, "parse", cmd.hex()) == \
        "CMD seq=200 left=1.500000 right=-0.750000 flags=1"
    cfg = P.Config(10240.0, 0.5, 0.25, 0.125, 20.0, 250, 5, 10)
    out = run(host_bin, "parse", P.encode_frame(P.MSG_CONFIG_SET, cfg.to_payload(), 3).hex())
    assert out == ("CFG seq=3 tpr=10240.000000 kp=0.500000 ki=0.250000 kd=0.125000 "
                   "max=20.000000 wd=250 einv=5 minv=10")


def test_cpp_decodes_python_servo_frame(host_bin):
    f = P.encode_frame(P.MSG_CMD_SERVOS, P.pack_cmd_servos(105, -3), seq=11)
    assert run(host_bin, "parse", f.hex()) == "SERVOS seq=11 s1=105 s2=-3"


def test_cpp_rejects_corrupted_frame(host_bin):
    f = bytearray(P.encode_frame(P.MSG_CMD_WHEELS, P.pack_cmd_wheels(1.0, 1.0), 1))
    f[7] ^= 0x55
    assert run(host_bin, "parse", bytes(f).hex()).startswith("NOFRAME crc_errors=1")
