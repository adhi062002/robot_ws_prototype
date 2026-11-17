// Closed-loop simulation test for teensy41_ugv.ino.
// Runs the REAL sketch code on the PC against:
//   * a simulated DC motor per wheel (first-order lag, dead-band, lockable) driven through the
//     sketch's real IN_A/IN_B/EN pin writes, using YOUR pin map
//   * simulated AS5600 encoders behind a TCA9548A mux on Wire2 (12-bit wrap, NAK injection)
//   * a fake e-stop pin, fake Jetson link (real ugv_protocol frames), fake Nano IMU stream
// It checks control behaviour and every safety rule. It does NOT model your real motors:
// PID gains tuned here are only a starting point.
#include <Arduino.h>
#include <Servo.h>
#include <TeensyTimerTool.h>
#include <Wire.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <vector>

static uint64_t g_us = 0;
static std::map<int, int> g_dig, g_pwm;
static std::deque<int> g_rx, g_imu_rx;
static std::vector<uint8_t> g_tx;

uint32_t millis() { return (uint32_t)(g_us / 1000); }
uint32_t micros() { return (uint32_t)g_us; }
void delay(uint32_t) {}
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t p, uint8_t v) { g_dig[p] = v; }
int digitalRead(uint8_t p) { return g_dig[p]; }
void analogWrite(uint8_t p, int v) { g_pwm[p] = v; }
int analogRead(uint8_t) { return 0; }
void analogWriteResolution(int) {}
void analogWriteFrequency(uint8_t, uint32_t) {}
HardwareSerial Serial, Serial1;
void HardwareSerial::begin(uint32_t) {}
int HardwareSerial::available() {
  return this == &Serial ? (int)g_rx.size() : (int)g_imu_rx.size();
}
int HardwareSerial::read() {
  std::deque<int>& q = (this == &Serial) ? g_rx : g_imu_rx;
  if (q.empty()) return -1;
  int b = q.front(); q.pop_front(); return b;
}
size_t HardwareSerial::write(const uint8_t* p, size_t n) {
  if (this == &Serial) g_tx.insert(g_tx.end(), p, p + n);
  return n;
}
size_t HardwareSerial::write(uint8_t) { return 1; }
void HardwareSerial::println(const char*) {}
size_t HardwareSerial::readBytesUntil(char, char*, size_t) { return 0; }
void HardwareSerial::setTimeout(unsigned long) {}
static int g_servo_angle[2] = {0, 90};
static int g_servo_next = 0;
void Servo::attach(int) { id_ = g_servo_next++; }
void Servo::write(int a) { g_servo_angle[id_] = a; }
int Servo::read() { return g_servo_angle[id_]; }
TeensyTimerTool::PeriodicTimer::PeriodicTimer(TeensyTimerTool::TimerGenerator) {}
void TeensyTimerTool::PeriodicTimer::begin(void (*)(), unsigned long) {}

// ---------------------------- simulated motors + encoders ----------------------------
#include "../teensy41_ugv/ugv_protocol.h"

struct SimWheel { double w = 0, th = 0; bool blocked = false; };   // physical (+ = IN_A high)
static SimWheel SW[4];
constexpr double W_FULL = 25.0;       // rad/s at 100 % duty
constexpr double TAU = 0.15;          // s
constexpr double DEADBAND_DUTY = 0.05;
static double g_tpr = 10240.0;        // encoder ticks per wheel revolution (4096 * 2.5)
static bool g_nak[4] = {false, false, false, false};

// Left wheels (0, 2) are mounted mirrored: robot-forward = physical negative.
static bool is_left(int i) { return i == 0 || i == 2; }
static double fwd_w(int i) { return is_left(i) ? -SW[i].w : SW[i].w; }

static uint8_t g_mux = 0, g_addr = 0;
static std::deque<uint8_t> g_i2c_rx;
static int mux_channel() {
  for (int c = 0; c < 8; c++) if (g_mux == (1u << c)) return c;
  return -1;
}
static uint16_t raw_angle(int ch) {
  double ticks = SW[ch].th / (2.0 * M_PI) * g_tpr;
  long long t = (long long)std::floor(ticks);
  return (uint16_t)(((t % 4096) + 4096) % 4096);
}
TwoWire Wire, Wire1, Wire2;
void TwoWire::begin() {}
void TwoWire::setClock(uint32_t) {}
void TwoWire::beginTransmission(uint8_t a) { g_addr = a; }
size_t TwoWire::write(uint8_t b) { if (g_addr == 0x70) g_mux = b; return 1; }
uint8_t TwoWire::endTransmission(bool) {
  if (g_addr == 0x36) { int c = mux_channel(); if (c < 0 || c > 3 || g_nak[c]) return 2; }
  return 0;
}
uint8_t TwoWire::requestFrom(int a, int) {
  if (a != 0x36) return 0;
  int c = mux_channel();
  if (c < 0 || c > 3 || g_nak[c]) return 0;
  uint16_t r = raw_angle(c);
  g_i2c_rx.push_back(0x20);                       // STATUS: magnet detected
  g_i2c_rx.push_back((uint8_t)(r >> 8));
  g_i2c_rx.push_back((uint8_t)(r & 0xFF));
  return 3;
}
int TwoWire::read() {
  if (g_i2c_rx.empty()) return -1;
  int b = g_i2c_rx.front(); g_i2c_rx.pop_front(); return b;
}

#include SKETCH_PATH

// ---- plant (uses the sketch's own pin table, so the test follows any pin change) ----
static void physics(double dt) {
  for (int i = 0; i < 4; i++) {
    int ha = g_dig[MOTORS[i].in_a], hb = g_dig[MOTORS[i].in_b];
    int dir = (ha && !hb) ? 1 : ((!ha && hb) ? -1 : 0);
    double duty = g_pwm[MOTORS[i].en] / 4095.0;
    double target = (dir != 0 && duty > DEADBAND_DUTY) ? dir * duty * W_FULL : 0.0;
    if (SW[i].blocked) SW[i].w = 0;
    else SW[i].w += (target - SW[i].w) * (dt / TAU);
    SW[i].th += SW[i].w * dt;
  }
}

// ------------------------------------ harness ------------------------------------
static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s  [%s]\n", msg, #cond); failures++; } } while (0)

static ugv::Parser g_tp;
static ugv::Telemetry g_tel;
static bool g_have_tel = false;
static int g_tel_count = 0;
static uint8_t g_seq = 0;
static std::function<void()> g_each_ms;

static void drain_tx() {
  for (uint8_t b : g_tx) {
    if (g_tp.feed(b) && g_tp.type() == ugv::MSG_TELEMETRY && g_tp.len() == sizeof(ugv::Telemetry)) {
      memcpy(&g_tel, g_tp.payload(), sizeof g_tel);
      g_have_tel = true;
      g_tel_count++;
    }
  }
  g_tx.clear();
}
static void send(uint8_t type, const void* p, uint8_t len) {
  uint8_t buf[ugv::MAX_PAYLOAD + ugv::FRAME_OVERHEAD];
  size_t n = ugv::encode_frame(type, g_seq++, p, len, buf);
  for (size_t i = 0; i < n; i++) g_rx.push_back(buf[i]);
}
static void send_cmd(float l, float r, bool en = true) {
  ugv::CmdWheels c{l, r, (uint8_t)(en ? ugv::CMD_ENABLE : 0)};
  send(ugv::MSG_CMD_WHEELS, &c, sizeof c);
}
static void send_config(uint8_t enc_inv = 0b0101, uint8_t mot_inv = 0b0101,
                        float kp = 0.05f, float ki = 0.3f, float kd = 0.0f) {
  ugv::ConfigSet c{(float)g_tpr, kp, ki, kd, 20.0f, 250, enc_inv, mot_inv};
  send(ugv::MSG_CONFIG_SET, &c, sizeof c);
}
static void run_ms(int ms) {
  for (int t = 0; t < ms; t++) {
    g_us += 1000;
    physics(0.001);
    updateServoMovement();            // the real board calls this from a 1 ms timer
    if (g_each_ms) g_each_ms();
    loop();
    if (t % 5 == 4) drain_tx();
  }
  drain_tx();
}
static void drive(int ms, float l, float r, bool en = true) {
  int left = ms;
  while (left > 0) {
    send_cmd(l, r, en);
    int chunk = left < 20 ? left : 20;
    run_ms(chunk);
    left -= chunk;
  }
}
static bool all_pwm_zero() {
  for (int i = 0; i < 4; i++) if (g_pwm[MOTORS[i].en] != 0) return false;
  return true;
}
static bool st(uint16_t bit) { return (g_tel.status & bit) != 0; }
__attribute__((unused)) static double avg_fwd_over(int ms, int wheel, float l, float r) {
  double sum = 0; int n = 0;
  for (int t = 0; t < ms; t += 10) { drive(10, l, r); sum += fwd_w(wheel); n++; }
  return sum / n;
}

static void imu_packet(float ax, float ay, float az, float gx, float gy, float gz, bool corrupt = false,
                       bool push = true, std::vector<uint8_t>* out = nullptr) {
  uint8_t pkt[26];
  pkt[0] = '$';
  float v[6] = {ax, ay, az, gx, gy, gz};
  memcpy(pkt + 1, v, sizeof v);
  uint8_t x = 0;
  for (int i = 1; i < 25; i++) x ^= pkt[i];
  pkt[25] = x;
  if (corrupt) pkt[10] ^= 0x5A;
  for (int i = 0; i < 26; i++) {
    if (push) g_imu_rx.push_back(pkt[i]);
    if (out) out->push_back(pkt[i]);
  }
}

static const double EXP_L = ENCODERS_FITTED ? 10.0 : 12.5;

static void check_pin_map() {
  // Every pin used by the sketch must be unique, off the reserved UART/I2C lines, and each
  // motor PWM pin must be PWM-capable on a Teensy 4.1 (conservative list of sure ones).
  std::map<int, int> use;
  auto add = [&](int pin, const char* what) {
    if (pin < 0) return;
    use[pin]++;
    char m[96]; snprintf(m, sizeof m, "pin %d (%s) is used more than once", pin, what);
    CHECK(use[pin] == 1, m);
  };
  for (int i = 0; i < 4; i++) { add(MOTORS[i].in_a, "IN_A"); add(MOTORS[i].in_b, "IN_B"); add(MOTORS[i].en, "PWM"); }
  add(SERVO1_PIN, "servo1"); add(SERVO2_PIN, "servo2"); add(ESTOP_PIN, "e-stop");
  add(0, "IMU RX1"); add(1, "IMU TX1 (reserved)");
#if ENCODERS_FITTED
  add(ENC_SDA_PIN, "encoder SDA"); add(ENC_SCL_PIN, "encoder SCL");
#endif
  static const int pwm_ok[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,18,19,22,23,24,25,28,29,33,36,37};
  for (int i = 0; i < 4; i++) {
    bool ok = false;
    for (int p : pwm_ok) ok |= (p == MOTORS[i].en);
    char m[80]; snprintf(m, sizeof m, "motor %d enable pin %d must be PWM-capable", i, MOTORS[i].en);
    CHECK(ok, m);
  }
}

int main() {
  check_pin_map();
  g_dig[ESTOP_PIN] = HIGH;                      // e-stop released (pulled up)
  setup();
  run_ms(50);

  // ---- boot state ----
  CHECK(all_pwm_zero(), "motors are off after boot");
  CHECK(g_servo_angle[0] == 0 && g_servo_angle[1] == 90, "servos start at 0 / 90 like the old sketch");

  // ---- no config -> no motion, even with enable + commands ----
  drive(500, 10, 10);
  CHECK(all_pwm_zero(), "no CONFIG_SET -> motors stay off");
  CHECK(!st(ugv::ST_ARMED) && !st(ugv::ST_CONFIG_VALID), "not armed / config invalid before CONFIG_SET");

  // ---- corrupted frame must be ignored, valid one accepted ----
  {
    uint8_t buf[32]; ugv::CmdWheels c{99.0f, 99.0f, 1};
    size_t n = ugv::encode_frame(ugv::MSG_CMD_WHEELS, 1, &c, sizeof c, buf);
    buf[7] ^= 0xFF;
    for (size_t i = 0; i < n; i++) g_rx.push_back(buf[i]);
    g_rx.push_back(0x00); g_rx.push_back(0xA5);
    run_ms(10);
    CHECK(target_rad_s[0] != 99.0f, "corrupted command frame must be ignored");
  }

  // ---- config + closed-loop tracking ----
  send_config();
  run_ms(20);
  CHECK(st(ugv::ST_CONFIG_VALID), "config accepted");
  drive(2500, 10, 6);
  CHECK(st(ugv::ST_ARMED), "armed once config + enable + heartbeat present");
#if ENCODERS_FITTED
  for (int i = 0; i < 4; i++) {
    double tgt = is_left(i) ? 10 : 6;
    double a = avg_fwd_over(200, i, 10, 6);
    char m[80]; snprintf(m, sizeof m, "wheel %d tracks target (%.2f vs %.1f)", i, a, tgt);
    CHECK(std::fabs(a - tgt) < 0.4, m);
    CHECK(std::fabs(g_tel.wheel_vel[i] - tgt) < 0.6, "telemetry wheel_vel matches target");
  }
  for (int i = 0; i < 4; i++) {          // encoder unwrap: many 4096-tick wraps, ticks == truth
    double fwd_ticks = (is_left(i) ? -1.0 : 1.0) * std::floor(SW[i].th / (2 * M_PI) * g_tpr);
    char m[80]; snprintf(m, sizeof m, "wheel %d ticks follow truth across wraps (%d vs %.0f)",
                         i, g_tel.enc_ticks[i], fwd_ticks);
    CHECK(std::fabs(g_tel.enc_ticks[i] - fwd_ticks) < 25, m);   // telemetry lags a few ms
    CHECK(std::fabs(fwd_ticks) > 3 * 4096, "test really covered several 12-bit wraps");
  }
  CHECK(g_tel.enc_ok_mask == 0x0F, "all encoders healthy");

#else
  CHECK(st(ugv::ST_OPEN_LOOP), "open-loop build reports ST_OPEN_LOOP");
  CHECK(!st(ugv::ST_ENC_FAULT) && !st(ugv::ST_MOTOR_FAULT), "open loop is a mode, not a fault");
  CHECK(g_tel.enc_ok_mask == 0, "no encoders reported in open loop");
  CHECK(g_pwm[MOTORS[0].en] == 2047 && g_pwm[MOTORS[2].en] == 2047, "left PWM = feed-forward duty 0.5 (10 / 20 rad/s)");
  CHECK(g_pwm[MOTORS[1].en] == 1228 && g_pwm[MOTORS[3].en] == 1228, "right PWM = feed-forward duty 0.3 (6 / 20 rad/s)");
  CHECK(std::fabs(fwd_w(0) - 12.5) < 0.6 && std::fabs(fwd_w(1) - 7.5) < 0.6,
        "open-loop speed = duty x full speed (no correction)");
#endif

  // ---- telemetry rate ----
  { int c0 = g_tel_count; drive(1000, 10, 6); int d = g_tel_count - c0;
    CHECK(d >= 95 && d <= 105, "telemetry about 100 Hz"); }

  // ---- watchdog ----
  run_ms(400);
  CHECK(all_pwm_zero(), "watchdog: motors off when heartbeats stop");
  CHECK(st(ugv::ST_CMD_WATCHDOG) && !st(ugv::ST_ARMED), "watchdog flagged in telemetry");
  drive(1500, 10, 6);
  CHECK(st(ugv::ST_ARMED) && !st(ugv::ST_CMD_WATCHDOG), "watchdog clears when heartbeats resume");
  CHECK(std::fabs(fwd_w(0) - EXP_L) < 0.6, "speed recovers after watchdog");

  // ---- e-stop: latches; CLEAR_FAULT ignored while pressed ----
  g_dig[ESTOP_PIN] = LOW;
  drive(60, 10, 6);
  CHECK(all_pwm_zero() && st(ugv::ST_ESTOP), "e-stop stops motors and is reported");
  send(ugv::MSG_CLEAR_FAULT, nullptr, 0);
  drive(100, 10, 6);
  CHECK(all_pwm_zero() && st(ugv::ST_ESTOP), "CLEAR_FAULT is ignored while e-stop is pressed");
  g_dig[ESTOP_PIN] = HIGH;
  drive(300, 10, 6);
  CHECK(all_pwm_zero(), "e-stop stays latched after release until cleared");
  send(ugv::MSG_CLEAR_FAULT, nullptr, 0);
  drive(1500, 10, 6);
  CHECK(!st(ugv::ST_ESTOP) && st(ugv::ST_ARMED) && std::fabs(fwd_w(2) - EXP_L) < 0.6,
        "after release + CLEAR_FAULT the robot runs again");

#if ENCODERS_FITTED
  // ---- stall: blocked wheel -> latched MOTOR_FAULT, all motors off ----
  SW[0].blocked = true;
  drive(2000, 10, 6);
  CHECK(st(ugv::ST_MOTOR_FAULT), "blocked wheel latches MOTOR_FAULT");
  CHECK(all_pwm_zero(), "stall fault stops all four motors");
  SW[0].blocked = false;
  send(ugv::MSG_CLEAR_FAULT, nullptr, 0);
  drive(1800, 10, 6);
  CHECK(!st(ugv::ST_MOTOR_FAULT) && std::fabs(fwd_w(0) - 10) < 0.6, "recovers after CLEAR_FAULT");

  // ---- encoder I2C glitch: 10 ms is tolerated, 40 ms faults, recovery is automatic ----
  bool stopped_during_glitch = false;
  g_each_ms = [&]() { if (all_pwm_zero()) stopped_during_glitch = true; };
  g_nak[1] = true; drive(10, 10, 6); g_nak[1] = false; drive(300, 10, 6);
  g_each_ms = nullptr;
  CHECK(!stopped_during_glitch, "motors never cut out during a 10 ms encoder glitch");
  CHECK(st(ugv::ST_ARMED) && !st(ugv::ST_MOTOR_FAULT), "a 10 ms encoder glitch does not fault the robot");
  g_nak[1] = true; drive(60, 10, 6);
  CHECK(all_pwm_zero() && st(ugv::ST_ENC_FAULT), "a sustained encoder failure stops the motors");
  g_nak[1] = false; drive(1500, 10, 6);
  CHECK(st(ugv::ST_ARMED) && !st(ugv::ST_ENC_FAULT), "motors resume after the encoder recovers");

  // ---- wrong encoder direction must NOT run away: fault within ~1.5 s ----
  send_config(/*enc_inv=*/0b0000);                 // left encoders now have the wrong sign
  run_ms(20);
  double max_fwd = 0;
  g_each_ms = [&]() { for (int i = 0; i < 4; i++) max_fwd = std::fmax(max_fwd, std::fabs(fwd_w(i))); };
  drive(2500, 10, 6);
  g_each_ms = nullptr;
  CHECK(st(ugv::ST_MOTOR_FAULT) && all_pwm_zero(), "wrong encoder sign is caught (no endless runaway)");
  send_config(0b0101);
  run_ms(20);
  send(ugv::MSG_CLEAR_FAULT, nullptr, 0);
  drive(1800, 10, 6);
  CHECK(!st(ugv::ST_MOTOR_FAULT) && std::fabs(fwd_w(0) - 10) < 0.6, "recovers with the right config");

  // ---- reversal through zero must not trip the runaway check ----
  drive(800, 10, 10);
  drive(1500, -10, -10);
  CHECK(!st(ugv::ST_MOTOR_FAULT) && fwd_w(0) < -9.0, "direction reversal is not a fault");
  drive(500, 0, 0);

#else
  // Open loop has no feedback, so stall / runaway cannot be detected: documented limitation.
  SW[0].blocked = true;
  drive(1500, 10, 6);
  CHECK(!st(ugv::ST_MOTOR_FAULT) && st(ugv::ST_ARMED), "open loop: no stall detection (by design)");
  SW[0].blocked = false;
  drive(500, 0, 0);
#endif

  // ---- IMU: units, bad checksum, resync ----
  imu_packet(0.0f, 0.0f, 1.0f, 90.0f, 0.0f, -45.0f);
  drive(40, 0, 0);
  CHECK(std::fabs(g_tel.accel[2] - 9.80665f) < 1e-3f, "accel g -> m/s^2");
  CHECK(std::fabs(g_tel.gyro[0] - (float)M_PI / 2) < 1e-3f && std::fabs(g_tel.gyro[2] + (float)M_PI / 4) < 1e-3f,
        "gyro deg/s -> rad/s");
  CHECK(!st(ugv::ST_IMU_STALE), "IMU fresh");
  imu_packet(0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.0f, /*corrupt=*/true);
  drive(40, 0, 0);
  CHECK(std::fabs(g_tel.accel[2] - 9.80665f) < 1e-3f, "bad IMU checksum is rejected");
  for (int i = 0; i < 5; i++) g_imu_rx.push_back(0x01);            // junk before a real packet
  g_imu_rx.push_back('$'); for (int i = 0; i < 5; i++) g_imu_rx.push_back(0x02);   // false start
  imu_packet(0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  drive(40, 0, 0);
  CHECK(std::fabs(g_tel.accel[0] - 0.5f * 9.80665f) < 1e-3f, "IMU resyncs after a false '$'");
  drive(300, 0, 0);
  CHECK(st(ugv::ST_IMU_STALE), "IMU stale flag after silence");

  // ---- servos: clamped to limits, smooth, reaches target ----
  { ugv::CmdServos c{200, -5}; send(ugv::MSG_CMD_SERVOS, &c, sizeof c); }
  run_ms(20);
  CHECK(isMovingServos && targetAngle1 == 105 && targetAngle2 == 0, "servo targets clamped to limits");
  run_ms(1200);
  CHECK(!isMovingServos && g_servo_angle[0] == 105 && g_servo_angle[1] == 0, "servos reach the clamped targets");

  printf(failures ? "RESULT: %d FAILED\n" : "RESULT: all closed-loop simulation checks passed\n", failures);
  return failures ? 1 : 0;
}
