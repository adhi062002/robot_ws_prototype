// teensy41_ugv.ino -- wheel-level controller for the Orion UGV (protocol version).
//
// Built on your working Teensy_code_dc_motor.ino: SAME driver scheme (IN_A/IN_B + PWM enable
// per motor), SAME servo smoothing, SAME IMU packet from the Nano (230400 baud, '$' + 6
// floats + XOR), but with your CURRENT motor pins (26-37). New on top of that:
//   * framed binary protocol with CRC (ugv_protocol.h) instead of text lines
//   * closed-loop wheel-velocity PID with 4 AS5600 encoders (optional, see ENCODERS_FITTED)
//   * the IMU is parsed and time-stamped here instead of being relayed raw
//   * safety: command watchdog, e-stop input, stall latch, "no config -> no motion"
//
// STATUS: logic is tested on a PC against a simulated motor + encoder (firmware/test/
// ugv_sim_test.cpp). It has NOT been run on a real Teensy. Check every pin against your
// wiring and flash with the wheels OFF THE GROUND first.
//
// Wheel order everywhere: 0=FL 1=FR 2=RL 3=RR.  Left side = FL+RL, right side = FR+RR.

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <TeensyTimerTool.h>
#include "ugv_protocol.h"

using namespace TeensyTimerTool;
using namespace ugv;

// ===================== wiring (your current Teensy 4.1 motor wiring) ==============
struct MotorPins { uint8_t in_a, in_b, en; };
// Wheel order: 0=FL 1=FR 2=RL 3=RR.  Your channels: A: PWM 33, IN 34/35   B: PWM 36, IN 37/32
//                                                    C: PWM 29, IN 30/31   D: PWM 28, IN 27/26
// ASSUMED mapping (continues your old layout: first driver = left side, front then rear):
//   A = front-left, B = rear-left, C = front-right, D = rear-right.
// If a wheel turns when another should, swap the rows here. Verify with the wheels lifted.
constexpr MotorPins MOTORS[4] = {
  {34, 35, 33},   // 0 FL = motor A
  {30, 31, 29},   // 1 FR = motor C
  {37, 32, 36},   // 2 RL = motor B
  {27, 26, 28},   // 3 RR = motor D
};
// Forward = IN_A HIGH / IN_B LOW. Your old LEFT_INVERT = -1 is motor_invert_mask bits 0 and 2
// (0b0101), sent by the Jetson in CONFIG_SET. If a wheel spins the wrong way, flip its bit.
constexpr uint8_t DEFAULT_MOTOR_INVERT = 0b0101;

constexpr int SERVO1_PIN = 7, SERVO2_PIN = 8;
const int SERVO1_MIN_ANGLE = 0, SERVO1_MAX_ANGLE = 105;
const int SERVO2_MIN_ANGLE = 0, SERVO2_MAX_ANGLE = 180;

#define LINK Serial                 // Jetson (USB serial)
#define IMU_UART Serial1            // Nano 33 BLE IMU -> Teensy RX1 (pin 0)
constexpr uint32_t IMU_BAUD = 230400;

// ===================== encoders ====================================================
// 1 = four AS5600 encoders behind a TCA9548A mux are installed -> closed-loop PID.
// 0 = not installed yet -> OPEN LOOP: duty = target / max_wheel_rad_s, no PID, no stall
//     detection; telemetry carries ST_OPEN_LOOP so the Jetson knows speeds are not measured.
#ifndef ENCODERS_FITTED
#define ENCODERS_FITTED 1
#endif
// Encoder bus. Wire = SDA 18 / SCL 19, Wire2 = SDA 25 / SCL 24 (Teensy 4.1; verify on your
// pinout card). Neither collides with your motor pins; use whichever you have wired.
#define ENC_WIRE Wire2
constexpr int ENC_SDA_PIN = 25, ENC_SCL_PIN = 24;   // must match ENC_WIRE (Wire = 18 / 19)
constexpr uint8_t TCA_ADDR = 0x70;
constexpr uint8_t AS5600_ADDR = 0x36;
constexpr uint8_t ENC_MUX_CH[4] = {0, 1, 2, 3};   // mux channel of each wheel's encoder

constexpr int ESTOP_PIN = 38;       // pulled up; LOW = pressed. -1 to disable. (32 is a motor pin)
constexpr int BATT_PIN = -1;        // pin 14 (A0) is a motor pin here; use another ADC pin
constexpr float BATT_MV_PER_COUNT = 0.0f;
constexpr uint16_t BATT_LOW_MV = 9600;
// ===============================================================================

constexpr uint32_t CONTROL_PERIOD_US = 5000;      // 200 Hz
constexpr uint8_t TELEMETRY_DIVIDER = 2;          // 100 Hz
constexpr uint32_t IMU_STALE_MS = 100;
constexpr float TWO_PI_F = 6.2831853f;
constexpr float DEG2RAD = 0.017453293f;
constexpr float G_TO_MS2 = 9.80665f;
constexpr float VEL_LPF_ALPHA = 0.3f;
constexpr float STALL_DUTY = 0.35f;
constexpr float STALL_SPEED_RAD_S = 0.05f;
constexpr uint32_t STALL_MS = 1000;
// Wrong encoder direction (enc_invert_mask) turns the PID into positive feedback and drives
// the motor to full power. If the wheel is driven toward a target but measured motion is the
// OPPOSITE way for this long, latch MOTOR_FAULT. Also latch on gross overspeed.
constexpr float RUNAWAY_MIN_TARGET = 1.0f;       // rad/s
constexpr float RUNAWAY_MIN_OPPOSITE = 0.5f;     // rad/s
constexpr uint32_t RUNAWAY_MS = 1000;
constexpr float OVERSPEED_FACTOR = 1.5f;         // x max_wheel_rad_s
constexpr uint32_t OVERSPEED_MS = 200;
constexpr uint8_t ENC_FAIL_LIMIT = 3;             // ~15 ms of bad reads before a fault

// ------------------------------ state ------------------------------------
static ConfigSet cfg = {4096.0f, 0.02f, 0.10f, 0.0f, 20.0f, 250, 0, DEFAULT_MOTOR_INVERT};
static bool config_valid = false;       // no motion until the Jetson has sent CONFIG_SET
static bool cmd_enable = false;
static float target_rad_s[4] = {0, 0, 0, 0};
static uint32_t last_cmd_ms = 0;
static bool ever_got_cmd = false;

static int32_t ticks[4] = {0, 0, 0, 0};
#if ENCODERS_FITTED
static uint16_t raw_prev[4];
static bool raw_init[4] = {false, false, false, false};
static int32_t ticks_prev[4] = {0, 0, 0, 0};
static uint8_t enc_fail_count[4] = {0, 0, 0, 0};
#endif
static float vel[4] = {0, 0, 0, 0};
static uint8_t enc_ok_mask = 0;         // starts 0: not armed until all encoders read OK

static float integ[4] = {0, 0, 0, 0};
static float prev_err[4] = {0, 0, 0, 0};
static float duty_out[4] = {0, 0, 0, 0};
static uint32_t stall_since_ms[4] = {0, 0, 0, 0};
static uint32_t runaway_since_ms[4] = {0, 0, 0, 0};
static uint32_t overspeed_since_ms[4] = {0, 0, 0, 0};
static bool fault_estop = false, fault_motor = false;

static float gyro[3] = {0, 0, 0}, accel[3] = {0, 0, 0};
static uint32_t imu_t_us = 0, imu_last_ms = 0, imu_bad_packets = 0;
constexpr size_t IMU_PKT_SIZE = 26;     // '$' + 6 floats + XOR checksum (as your Nano sends)
static uint8_t imu_buf[IMU_PKT_SIZE];
static size_t imu_pos = 0;

static Parser parser;
static uint8_t tx_seq = 0;
static uint8_t tel_div = 0;
static uint32_t next_control_us = 0;

// ------------------------------ servos (your smoothing code, unchanged) --------------
Servo servo1;
Servo servo2;
int targetAngle1 = 0, targetAngle2 = 90;
int startAngle1 = 0, startAngle2 = 90;
unsigned long servoMoveStartTime = 0;
float servoMoveDuration = 0;
volatile bool isMovingServos = false;
PeriodicTimer servoTimer(GPT1);

void startSmoothMove(int t1, int t2) {
  if (isMovingServos) return;
  targetAngle1 = constrain(t1, SERVO1_MIN_ANGLE, SERVO1_MAX_ANGLE);
  targetAngle2 = constrain(t2, SERVO2_MIN_ANGLE, SERVO2_MAX_ANGLE);
  startAngle1 = servo1.read();
  startAngle2 = servo2.read();
  int maxDelta = max(abs(targetAngle1 - startAngle1), abs(targetAngle2 - startAngle2));
  if (maxDelta == 0) return;
  servoMoveDuration = constrain(maxDelta * 10.0, 30.0, 800.0);
  servoMoveStartTime = millis();
  isMovingServos = true;
}

void updateServoMovement() {
  if (!isMovingServos) return;
  float elapsed = millis() - servoMoveStartTime;
  if (elapsed >= servoMoveDuration) {
    servo1.write(targetAngle1);
    servo2.write(targetAngle2);
    isMovingServos = false;
    return;
  }
  float k = 12.0 / (servoMoveDuration / 1000.0);
  float t0 = servoMoveDuration / 2.0;
  float s = 1.0 / (1.0 + exp(-k * ((elapsed - t0) / 1000.0)));
  servo1.write(round(startAngle1 + (targetAngle1 - startAngle1) * s));
  servo2.write(round(startAngle2 + (targetAngle2 - startAngle2) * s));
}

// ------------------------------ motors -----------------------------------
// duty in [-1, 1]. Same pin behaviour as your driveSide(): forward = IN_A HIGH / IN_B LOW,
// reverse = IN_A LOW / IN_B HIGH, zero = both LOW (coast) with PWM 0.
static void setMotor(int i, float duty) {
  duty = constrain(duty, -1.0f, 1.0f);
  const MotorPins& m = MOTORS[i];
  if (duty == 0.0f) {
    digitalWrite(m.in_a, LOW);
    digitalWrite(m.in_b, LOW);
    analogWrite(m.en, 0);
    return;
  }
  bool fwd = duty > 0.0f;
  if (cfg.motor_invert_mask & (1 << i)) fwd = !fwd;
  digitalWrite(m.in_a, fwd ? HIGH : LOW);
  digitalWrite(m.in_b, fwd ? LOW : HIGH);
  analogWrite(m.en, (int)(fabsf(duty) * 4095.0f));
}

static void stopAllMotors() {
  for (int i = 0; i < 4; i++) {
    duty_out[i] = 0;
    integ[i] = 0;
    prev_err[i] = 0;
    setMotor(i, 0.0f);
  }
}

// ------------------------------ encoders ---------------------------------
#if ENCODERS_FITTED
static bool muxSelect(uint8_t ch) {
  ENC_WIRE.beginTransmission(TCA_ADDR);
  ENC_WIRE.write((uint8_t)(1 << ch));
  return ENC_WIRE.endTransmission() == 0;
}

// Reads STATUS(0x0B) + RAW_ANGLE(0x0C,0x0D). Returns false on any bus error.
static bool readAS5600(uint8_t wheel, uint16_t* raw, bool* magnet_ok) {
  if (!muxSelect(ENC_MUX_CH[wheel])) return false;
  ENC_WIRE.beginTransmission(AS5600_ADDR);
  ENC_WIRE.write((uint8_t)0x0B);
  if (ENC_WIRE.endTransmission(false) != 0) return false;
  if (ENC_WIRE.requestFrom((int)AS5600_ADDR, 3) != 3) return false;
  uint8_t status = (uint8_t)ENC_WIRE.read();
  uint8_t hi = (uint8_t)ENC_WIRE.read();
  uint8_t lo = (uint8_t)ENC_WIRE.read();
  *raw = (uint16_t)(((hi & 0x0F) << 8) | lo);
  bool md = status & 0x20, ml = status & 0x10, mh = status & 0x08;  // detected/weak/strong
  *magnet_ok = md && !ml && !mh;
  return true;
}

static void updateEncoders() {
  for (uint8_t i = 0; i < 4; i++) {
    uint16_t raw;
    bool mag = false;
    bool ok = readAS5600(i, &raw, &mag) && mag;
    if (ok) {
      enc_fail_count[i] = 0;
    } else if (enc_fail_count[i] < 255) {
      enc_fail_count[i]++;                          // one glitch must not stop the robot
    }
    if (enc_fail_count[i] >= ENC_FAIL_LIMIT) enc_ok_mask = (uint8_t)(enc_ok_mask & ~(1 << i));
    else enc_ok_mask = (uint8_t)(enc_ok_mask | (1 << i));
    if (!ok) continue;                              // keep last ticks on a bad read
    if (!raw_init[i]) { raw_prev[i] = raw; raw_init[i] = true; continue; }
    int32_t d = (int32_t)raw - (int32_t)raw_prev[i];
    if (d > 2048) d -= 4096;                        // 12-bit wrap
    if (d < -2048) d += 4096;
    raw_prev[i] = raw;
    ticks[i] += (cfg.enc_invert_mask & (1 << i)) ? -d : d;
  }
}
#else
static void updateEncoders() {}
#endif

// ------------------------------ control ----------------------------------
static bool commandWatchdogTripped(uint32_t now_ms) {
  return !ever_got_cmd || (now_ms - last_cmd_ms) > cfg.watchdog_ms;
}

static bool isArmed(uint32_t now_ms) {
  bool enc_ok_all = !ENCODERS_FITTED || (enc_ok_mask == 0x0F);
  return config_valid && cmd_enable && !commandWatchdogTripped(now_ms) && !fault_estop &&
         !fault_motor && enc_ok_all;
}

static void controlStep(float dt, uint32_t now_ms) {
#if ENCODERS_FITTED
  const float rad_per_tick = TWO_PI_F / cfg.ticks_per_wheel_rev;
  for (int i = 0; i < 4; i++) {
    float raw_vel = (float)(ticks[i] - ticks_prev[i]) * rad_per_tick / dt;
    ticks_prev[i] = ticks[i];
    vel[i] += VEL_LPF_ALPHA * (raw_vel - vel[i]);
  }
#endif

  // E-stop input: latch until released AND cleared by the Jetson.
  if (ESTOP_PIN >= 0 && digitalRead(ESTOP_PIN) == LOW) fault_estop = true;

  if (!isArmed(now_ms)) {
    stopAllMotors();
    for (int i = 0; i < 4; i++) {
      stall_since_ms[i] = 0;
      runaway_since_ms[i] = 0;
      overspeed_since_ms[i] = 0;
    }
    return;
  }

  for (int i = 0; i < 4; i++) {
    float tgt = constrain(target_rad_s[i], -cfg.max_wheel_rad_s, cfg.max_wheel_rad_s);
    float ff = tgt / cfg.max_wheel_rad_s;                              // feed-forward
#if ENCODERS_FITTED
    float err = tgt - vel[i];
    integ[i] = constrain(integ[i] + cfg.ki * err * dt, -0.3f, 0.3f);   // anti-windup
    float deriv = (err - prev_err[i]) / dt;
    prev_err[i] = err;
    duty_out[i] = constrain(ff + cfg.kp * err + integ[i] + cfg.kd * deriv, -1.0f, 1.0f);
    // stall: strong drive, no motion
    if (fabsf(duty_out[i]) > STALL_DUTY && fabsf(vel[i]) < STALL_SPEED_RAD_S) {
      if (stall_since_ms[i] == 0) stall_since_ms[i] = now_ms;
      if (now_ms - stall_since_ms[i] > STALL_MS) fault_motor = true;
    } else {
      stall_since_ms[i] = 0;
    }
    // runaway: driven toward tgt but measured motion is the opposite way (bad encoder sign)
    if (fabsf(tgt) > RUNAWAY_MIN_TARGET && vel[i] * tgt < 0.0f &&
        fabsf(vel[i]) > RUNAWAY_MIN_OPPOSITE) {
      if (runaway_since_ms[i] == 0) runaway_since_ms[i] = now_ms;
      if (now_ms - runaway_since_ms[i] > RUNAWAY_MS) fault_motor = true;
    } else {
      runaway_since_ms[i] = 0;
    }
    // gross overspeed
    if (fabsf(vel[i]) > OVERSPEED_FACTOR * cfg.max_wheel_rad_s) {
      if (overspeed_since_ms[i] == 0) overspeed_since_ms[i] = now_ms;
      if (now_ms - overspeed_since_ms[i] > OVERSPEED_MS) fault_motor = true;
    } else {
      overspeed_since_ms[i] = 0;
    }
#else
    duty_out[i] = constrain(ff, -1.0f, 1.0f);                          // open loop
#endif
    setMotor(i, duty_out[i]);
  }
}

// ------------------------------ IMU --------------------------------------
// The Nano sends 26-byte packets: '$', ax ay az (g), gx gy gz (deg/s), XOR of bytes 1..24.
// A bad checksum re-syncs on the next '$' inside the buffer instead of dropping 26 bytes.
static void imuPushByte(uint8_t b) {
  if (imu_pos == 0 && b != '$') return;
  imu_buf[imu_pos++] = b;
  if (imu_pos < IMU_PKT_SIZE) return;

  uint8_t x = 0;
  for (size_t i = 1; i < IMU_PKT_SIZE - 1; i++) x ^= imu_buf[i];
  if (x == imu_buf[IMU_PKT_SIZE - 1]) {
    float v[6];
    memcpy(v, imu_buf + 1, sizeof v);               // both CPUs are little-endian
    for (int i = 0; i < 3; i++) {
      accel[i] = v[i] * G_TO_MS2;
      gyro[i] = v[3 + i] * DEG2RAD;
    }
    imu_t_us = micros();
    imu_last_ms = millis();
    imu_pos = 0;
    return;
  }
  imu_bad_packets++;
  size_t next = 0;
  for (size_t i = 1; i < IMU_PKT_SIZE; i++) {
    if (imu_buf[i] == '$') { next = i; break; }
  }
  if (next == 0) { imu_pos = 0; return; }
  memmove(imu_buf, imu_buf + next, IMU_PKT_SIZE - next);
  imu_pos = IMU_PKT_SIZE - next;
}

static void pollImu() {
  while (IMU_UART.available()) imuPushByte((uint8_t)IMU_UART.read());
}

// ------------------------------ link -------------------------------------
static void sendFrame(uint8_t type, const void* payload, uint8_t len) {
  uint8_t buf[MAX_PAYLOAD + FRAME_OVERHEAD];
  size_t n = encode_frame(type, tx_seq++, payload, len, buf);
  if (n) LINK.write(buf, n);
}

static void sendConfigReport() { sendFrame(MSG_CONFIG_REPORT, &cfg, sizeof cfg); }

static void sendTelemetry(uint32_t now_ms) {
  Telemetry t;
  memset(&t, 0, sizeof t);
  t.t_us = micros();
  for (int i = 0; i < 4; i++) { t.enc_ticks[i] = ticks[i]; t.wheel_vel[i] = vel[i]; }
  for (int i = 0; i < 3; i++) { t.gyro[i] = gyro[i]; t.accel[i] = accel[i]; }
  t.imu_t_us = imu_t_us;

  uint16_t batt_mv = 0;
  if (BATT_PIN >= 0) batt_mv = (uint16_t)((float)analogRead(BATT_PIN) * BATT_MV_PER_COUNT);

  uint16_t st = 0;
  if (fault_estop) st |= ST_ESTOP;
  if (commandWatchdogTripped(now_ms)) st |= ST_CMD_WATCHDOG;
  if ((now_ms - imu_last_ms) > IMU_STALE_MS) st |= ST_IMU_STALE;
#if ENCODERS_FITTED
  if (enc_ok_mask != 0x0F) st |= ST_ENC_FAULT;
#else
  st |= ST_OPEN_LOOP;
#endif
  if (fault_motor) st |= ST_MOTOR_FAULT;
  if (config_valid) st |= ST_CONFIG_VALID;
  if (BATT_PIN >= 0 && batt_mv > 0 && batt_mv < BATT_LOW_MV) st |= ST_LOW_BATTERY;
  if (isArmed(now_ms)) st |= ST_ARMED;
  t.status = st;

  uint32_t age = now_ms - last_cmd_ms;
  t.cmd_age_ms = ever_got_cmd ? (uint16_t)(age > 65535 ? 65535 : age) : 65535;
  t.batt_mv = batt_mv;
  t.enc_ok_mask = enc_ok_mask;
  sendFrame(MSG_TELEMETRY, &t, sizeof t);
}

static void handleFrame(uint32_t now_ms) {
  switch (parser.type()) {
    case MSG_CMD_WHEELS:
      if (parser.len() == sizeof(CmdWheels)) {
        CmdWheels c;
        memcpy(&c, parser.payload(), sizeof c);
        if (isfinite(c.left_rad_s) && isfinite(c.right_rad_s)) {   // reject NaN/inf
          target_rad_s[0] = target_rad_s[2] = c.left_rad_s;
          target_rad_s[1] = target_rad_s[3] = c.right_rad_s;
          cmd_enable = (c.flags & CMD_ENABLE) != 0;
          last_cmd_ms = now_ms;
          ever_got_cmd = true;
        }
      }
      break;
    case MSG_CMD_SERVOS:
      if (parser.len() == sizeof(CmdServos)) {
        CmdServos c;
        memcpy(&c, parser.payload(), sizeof c);
        startSmoothMove(c.servo1_deg, c.servo2_deg);   // clamped to the servo limits
      }
      break;
    case MSG_CONFIG_SET:
      if (parser.len() == sizeof(ConfigSet)) {
        ConfigSet c;
        memcpy(&c, parser.payload(), sizeof c);
        bool sane = c.ticks_per_wheel_rev > 100.0f && c.max_wheel_rad_s > 0.1f &&
                    c.watchdog_ms >= 50 && c.watchdog_ms <= 2000 &&
                    isfinite(c.kp) && isfinite(c.ki) && isfinite(c.kd) &&
                    isfinite(c.ticks_per_wheel_rev) && isfinite(c.max_wheel_rad_s);
        if (sane) {
          stopAllMotors();             // never change gains/units under a moving robot
          cfg = c;
          config_valid = true;
        }
        sendConfigReport();
      }
      break;
    case MSG_REQUEST_CONFIG:
      sendConfigReport();
      break;
    case MSG_CLEAR_FAULT:
      if (!(ESTOP_PIN >= 0 && digitalRead(ESTOP_PIN) == LOW)) fault_estop = false;
      fault_motor = false;
      break;
    default:
      break;
  }
}

// ------------------------------ Arduino entry points ---------------------
void setup() {
  LINK.begin(115200);                  // ignored for USB serial
  IMU_UART.begin(IMU_BAUD);

  for (int i = 0; i < 4; i++) {        // motors OFF before anything else happens
    pinMode(MOTORS[i].in_a, OUTPUT);
    pinMode(MOTORS[i].in_b, OUTPUT);
    pinMode(MOTORS[i].en, OUTPUT);
    setMotor(i, 0.0f);
  }
  analogWriteResolution(12);           // PWM frequency is left at the Teensy default, as before

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.write(targetAngle1);
  servo2.write(targetAngle2);
  servoTimer.begin(updateServoMovement, 1000);   // 1 ms, as in your old sketch

#if ENCODERS_FITTED
  ENC_WIRE.begin();
  ENC_WIRE.setClock(400000);
#endif
  if (ESTOP_PIN >= 0) pinMode(ESTOP_PIN, INPUT_PULLUP);
  next_control_us = micros() + CONTROL_PERIOD_US;
}

void loop() {
  uint32_t now_ms = millis();

  while (LINK.available()) {
    if (parser.feed((uint8_t)LINK.read())) handleFrame(now_ms);
  }
  pollImu();

  if ((int32_t)(micros() - next_control_us) >= 0) {
    next_control_us += CONTROL_PERIOD_US;
    updateEncoders();
    controlStep(CONTROL_PERIOD_US * 1e-6f, now_ms);
    if (++tel_div >= TELEMETRY_DIVIDER) {
      tel_div = 0;
      sendTelemetry(now_ms);
    }
  }
}
