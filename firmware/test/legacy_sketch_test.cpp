// Host test for Teensy_code_dc_motor_fixed.ino: runs the REAL sketch code against a fake
// serial port, fake pins and a fake clock. Verifies the command parser and the watchdog.
#include <Arduino.h>
#include <Servo.h>
#include <TeensyTimerTool.h>
#include <cstdio>
#include <deque>
#include <map>
#include <string>

static uint32_t g_ms = 0;
static std::map<int, int> g_dig, g_pwm;
static std::deque<int> g_rx;           // bytes "from the Orin"

uint32_t millis() { return g_ms; }
uint32_t micros() { return g_ms * 1000u; }
void delay(uint32_t ms) { g_ms += ms; }
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t p, uint8_t v) { g_dig[p] = v; }
int digitalRead(uint8_t p) { return g_dig[p]; }
void analogWrite(uint8_t p, int v) { g_pwm[p] = v; }
int analogRead(uint8_t) { return 0; }
void analogWriteResolution(int) {}
void analogWriteFrequency(uint8_t, uint32_t) {}
HardwareSerial Serial, Serial1;
void HardwareSerial::begin(uint32_t) {}
int HardwareSerial::available() { return this == &Serial ? (int)g_rx.size() : 0; }
int HardwareSerial::read() {
  if (this != &Serial || g_rx.empty()) return -1;
  int b = g_rx.front(); g_rx.pop_front(); return b;
}
size_t HardwareSerial::write(const uint8_t*, size_t n) { return n; }
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

#include SKETCH_PATH

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s  (%s)\n", msg, #cond); failures++; } } while (0)

static void feed(const std::string& s) { for (char c : s) g_rx.push_back((unsigned char)c); }
static void run(int ms) {            // advance time in 10 ms steps, calling loop()
  for (int t = 0; t < ms; t += 10) { g_ms += 10; loop(); }
}
static void reset_pins() { g_dig.clear(); g_pwm.clear(); }

int main() {
  setup();
  g_ms = 1000;

  // 1) a complete line delivered in small pieces is acted on only when the '\n' arrives
  reset_pins();
  feed("LEFT:100 RI"); run(20);
  CHECK(g_pwm[L_ENA] == 0 && g_pwm[R_ENA] == 0, "partial line must not move motors");
  feed("GHT:-100\n"); run(20);
  // LEFT_INVERT=-1: LEFT:100 -> -100 -> IN1 LOW, IN2 HIGH, PWM 100 on both left motors
  CHECK(g_pwm[L_ENA] == 100 && g_pwm[L_ENB] == 100, "left PWM");
  CHECK(g_dig[L_IN1] == LOW && g_dig[L_IN2] == HIGH && g_dig[L_IN3] == LOW && g_dig[L_IN4] == HIGH,
        "left direction (inverted)");
  // RIGHT_INVERT=+1: RIGHT:-100 -> -100 -> IN1 LOW, IN2 HIGH
  CHECK(g_pwm[R_ENA] == 100 && g_pwm[R_ENB] == 100, "right PWM");
  CHECK(g_dig[R_IN1] == LOW && g_dig[R_IN2] == HIGH, "right direction");

  // 2) an unterminated fragment alone never acts (old code would act after its 1 s timeout)
  reset_pins();
  feed("LEFT:200");
  run(1500);
  CHECK(g_pwm[L_ENA] == 0, "unterminated fragment must not drive motors");
  g_rx.clear();

  // 3) overlong garbage line is discarded entirely, and the next good line still works
  feed(std::string(400, 'x') + " LEFT:50\n");
  feed("LEFT:60 RIGHT:60\n"); run(20);
  CHECK(g_pwm[L_ENA] == 60 && g_pwm[R_ENA] == 60, "good line after overlong garbage");

  // 4) watchdog: no further commands -> motors stop after COMMAND_TIMEOUT_MS
  run(300);
  CHECK(g_pwm[L_ENA] == 60, "still driving before the timeout");
  run(400);
  CHECK(g_pwm[L_ENA] == 0 && g_pwm[L_ENB] == 0 && g_pwm[R_ENA] == 0 && g_pwm[R_ENB] == 0,
        "watchdog must stop all four motors");
  CHECK(g_dig[L_IN1] == LOW && g_dig[L_IN2] == LOW, "watchdog leaves driver inputs off");

  // 5) a refreshing command stream keeps the robot running past the timeout
  for (int i = 0; i < 20; i++) { feed("LEFT:80 RIGHT:80\n"); run(100); }
  CHECK(g_pwm[L_ENA] == 80, "continuous commands keep motors on (2 s > timeout)");

  // 6) explicit zero stops and does not get re-triggered
  feed("LEFT:0 RIGHT:0\n"); run(20);
  CHECK(g_pwm[L_ENA] == 0 && g_pwm[R_ENA] == 0, "explicit stop");

  // 7) servo tokens still parse (old behaviour preserved)
  feed("servo1:45 servo2:120\n"); run(20);
  CHECK(isMovingServos && targetAngle1 == 45 && targetAngle2 == 120, "servo command parsed");

  printf(failures ? "RESULT: %d FAILED\n" : "RESULT: all legacy-sketch checks passed\n", failures);
  return failures ? 1 : 0;
}
