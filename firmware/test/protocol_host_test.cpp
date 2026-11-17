// Host-side helper used by src/ugv_hardware/test/test_protocol.py.
//   protocol_host_test crc
//   protocol_host_test emit_telemetry        -> hex of a known Telemetry frame
//   protocol_host_test parse <hexframe>      -> decoded fields of CMD_WHEELS / CONFIG_SET
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include "../teensy41_ugv/ugv_protocol.h"

using namespace ugv;

static std::vector<uint8_t> from_hex(const char* s) {
  std::vector<uint8_t> v;
  for (size_t i = 0; s[i] && s[i + 1]; i += 2) {
    char b[3] = {s[i], s[i + 1], 0};
    v.push_back((uint8_t)strtoul(b, nullptr, 16));
  }
  return v;
}

int main(int argc, char** argv) {
  if (argc < 2) return 2;
  std::string mode = argv[1];
  if (mode == "crc") {
    printf("%04X\n", crc16((const uint8_t*)"123456789", 9));
  } else if (mode == "emit_telemetry") {
    Telemetry t{};
    t.t_us = 4000000000u;  // close to the u32 wrap on purpose
    int32_t ticks[4] = {100, -200, 300, -2147483647};
    float vel[4] = {1.5f, -2.25f, 0.5f, 0.0f};
    memcpy(t.enc_ticks, ticks, sizeof ticks);
    memcpy(t.wheel_vel, vel, sizeof vel);
    t.gyro[0] = 0.25f; t.gyro[1] = -0.5f; t.gyro[2] = 1.0f;
    t.accel[0] = 0.0f; t.accel[1] = 9.75f; t.accel[2] = -1.5f;
    t.imu_t_us = 3999990000u;
    t.status = ST_CONFIG_VALID | ST_ARMED;
    t.cmd_age_ms = 42;
    t.batt_mv = 11800;
    t.enc_ok_mask = 0x0F;
    uint8_t out[MAX_PAYLOAD + FRAME_OVERHEAD];
    size_t n = encode_frame(MSG_TELEMETRY, 7, &t, sizeof t, out);
    for (size_t i = 0; i < n; i++) printf("%02X", out[i]);
    printf("\n");
  } else if (mode == "parse" && argc >= 3) {
    Parser p;
    auto bytes = from_hex(argv[2]);
    bool got = false;
    for (uint8_t b : bytes) got = p.feed(b) || got;
    if (!got) { printf("NOFRAME crc_errors=%u bad_len=%u\n", p.crc_errors, p.bad_len); return 0; }
    if (p.type() == MSG_CMD_WHEELS && p.len() == sizeof(CmdWheels)) {
      CmdWheels c; memcpy(&c, p.payload(), sizeof c);
      printf("CMD seq=%u left=%.6f right=%.6f flags=%u\n", p.seq(), c.left_rad_s,
             c.right_rad_s, c.flags);
    } else if (p.type() == MSG_CMD_SERVOS && p.len() == sizeof(CmdServos)) {
      CmdServos c; memcpy(&c, p.payload(), sizeof c);
      printf("SERVOS seq=%u s1=%d s2=%d\n", p.seq(), c.servo1_deg, c.servo2_deg);
    } else if (p.type() == MSG_CONFIG_SET && p.len() == sizeof(ConfigSet)) {
      ConfigSet c; memcpy(&c, p.payload(), sizeof c);
      printf("CFG seq=%u tpr=%.6f kp=%.6f ki=%.6f kd=%.6f max=%.6f wd=%u einv=%u minv=%u\n",
             p.seq(), c.ticks_per_wheel_rev, c.kp, c.ki, c.kd, c.max_wheel_rad_s,
             c.watchdog_ms, c.enc_invert_mask, c.motor_invert_mask);
    } else {
      printf("OTHER type=%u len=%u seq=%u\n", p.type(), p.len(), p.seq());
    }
  } else {
    return 2;
  }
  return 0;
}
