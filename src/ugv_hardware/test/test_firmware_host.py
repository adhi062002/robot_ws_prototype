"""Host-side firmware checks, run through pytest so there is one command for everything.

They compile the real .ino files against an Arduino shim (firmware/test/shim) and run them
against a fake clock, fake pins, a simulated motor/encoder and fake serial links.
They prove the LOGIC; they do not replace a first test on the real Teensy.

Run:  cd src/ugv_hardware && python -m pytest test/test_firmware_host.py -q   (needs g++)
"""
import shutil
import subprocess
from pathlib import Path

import pytest

FW = Path(__file__).resolve().parents[3] / "firmware"
TEST = FW / "test"
SHIM = TEST / "shim"
V2 = FW / "teensy41_ugv" / "teensy41_ugv.ino"
LEGACY = FW / "Teensy_code_dc_motor_fixed" / "Teensy_code_dc_motor_fixed.ino"

pytestmark = pytest.mark.skipif(shutil.which("g++") is None, reason="g++ not available")


def _build_and_run(tmp_path, source, sketch, extra=()):
    exe = tmp_path / source.stem
    cmd = ["g++", "-std=c++17", "-Wall", "-Wextra", "-Wno-unused-parameter", f"-I{SHIM}",
           f'-DSKETCH_PATH="{sketch}"', *extra, str(source), "-o", str(exe)]
    b = subprocess.run(cmd, capture_output=True, text=True)
    assert b.returncode == 0, b.stderr[-2000:]
    r = subprocess.run([str(exe)], capture_output=True, text=True, timeout=600)
    assert r.returncode == 0, r.stdout[-2000:]
    return r.stdout


def test_fixed_legacy_sketch_parser_and_watchdog(tmp_path):
    out = _build_and_run(tmp_path, TEST / "legacy_sketch_test.cpp", LEGACY)
    assert "all legacy-sketch checks passed" in out


def test_v2_firmware_closed_loop_simulation(tmp_path):
    out = _build_and_run(tmp_path, TEST / "ugv_sim_test.cpp", V2, ["-DENCODERS_FITTED=1"])
    assert "all closed-loop simulation checks passed" in out


def test_v2_firmware_open_loop_simulation(tmp_path):
    out = _build_and_run(tmp_path, TEST / "ugv_sim_test.cpp", V2, ["-DENCODERS_FITTED=0"])
    assert "all closed-loop simulation checks passed" in out


@pytest.mark.parametrize("fitted", [0, 1])
def test_v2_firmware_strict_compile_both_variants(fitted):
    r = subprocess.run(
        ["g++", "-std=c++17", "-Wall", "-Wextra", "-Wshadow", "-Werror", "-Wno-unused-parameter",
         f"-DENCODERS_FITTED={fitted}", "-fsyntax-only", f"-I{SHIM}", f"-I{V2.parent}",
         "-x", "c++", str(V2)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr[-2000:]
