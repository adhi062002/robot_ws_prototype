#include <Servo.h>
#include <TeensyTimerTool.h>
using namespace TeensyTimerTool;

// Forward declarations (the Arduino IDE adds these itself; PlatformIO/g++ need them).
void stopMotors();
void handleOrinCommands();
void handleArduinoIMU();
void updateServoMovement();
void startSmoothMove(int t1, int t2);

// =================================================================
//                          HARDWARE SETUP
// =================================================================
// Pins updated to the current Teensy 4.1 wiring (motors A-D on pins 26-37). ASSUMED mapping:
// A = front-left, B = back-left (left driver), C = front-right, D = back-right (right driver).
// Verify with the wheels lifted; if a motor spins the wrong way swap its two IN pins, and
// if a wheel is on the wrong side swap the rows. The old pins are shown as "was ...".

// ----- SERVOS -----
Servo servo1;
Servo servo2;

// ----- LEFT DRIVER -----
// Front-Left motor
#define L_IN1 34   // motor A (was 14)
#define L_IN2 35   // (was 15)
#define L_ENA 33   // PWM (was 9)
#define LEFT_INVERT  -1


// Back-Left motor
#define L_IN3 37   // motor B (was 17)
#define L_IN4 32   // (was 16)
#define L_ENB 36   // PWM (was 10)


// ----- RIGHT DRIVER -----
// Front-Right motor
#define R_IN1 30   // motor C (was 18)
#define R_IN2 31   // (was 19)
#define R_ENA 29   // PWM (was 2)
#define RIGHT_INVERT  1
// Back-Right motor
#define R_IN3 27   // motor D (was 20)
#define R_IN4 26   // (was 21)
#define R_ENB 28   // PWM (was 3)

// =================================================================
//                        IMU PACKET STRUCT
// =================================================================
struct __attribute__((packed)) IMUDataPacket {
  char start_byte;
  float ax, ay, az;
  float gx, gy, gz;
  uint8_t checksum;
};

// =================================================================
//                 NON-BLOCKING SERVO MOVEMENT STATE
// =================================================================
int targetAngle1 = 0;
int targetAngle2 = 90;
int startAngle1  = 0;
int startAngle2  = 90;

unsigned long servoMoveStartTime = 0;
float servoMoveDuration = 0;
bool isMovingServos = false;

// Servo limits
const int SERVO1_MIN_ANGLE = 0;
const int SERVO1_MAX_ANGLE = 105;
const int SERVO2_MIN_ANGLE = 0;
const int SERVO2_MAX_ANGLE = 180;

// Timer
PeriodicTimer servoTimer(GPT1);

// =================================================================
//                    COMMAND WATCHDOG (safety)
// =================================================================
// If the Orin stops sending commands (crash, cable pulled, node killed) the motors
// are switched off. The Orin side must therefore keep sending, ideally >= 10 Hz,
// and should send LEFT:0 RIGHT:0 when it wants to stand still.
// Set to 0 to disable.
const unsigned long COMMAND_TIMEOUT_MS = 500;
unsigned long lastCommandTime = 0;
int currentLeft = 0;     // last LEFT value commanded (0 = stopped)
int currentRight = 0;

// =================================================================
//                              SETUP
// =================================================================
void setup() {
  Serial.begin(115200);      // Orin Nano
  Serial1.begin(230400);     // Arduino IMU

  // Servo pins
  servo1.attach(7);
  servo2.attach(8);

  servo1.write(targetAngle1);
  servo2.write(targetAngle2);

  // Motor pins
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(L_IN3, OUTPUT); pinMode(L_IN4, OUTPUT);
  pinMode(L_ENA, OUTPUT); pinMode(L_ENB, OUTPUT);

  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  pinMode(R_IN3, OUTPUT); pinMode(R_IN4, OUTPUT);
  pinMode(R_ENA, OUTPUT); pinMode(R_ENB, OUTPUT);

  // Servo update timer (1 ms)
  servoTimer.begin(updateServoMovement, 1000);

  delay(100);
  Serial.println("Teensy Controller Ready (DC Motors + Sigmoid Servos)");
}

// =================================================================
//                              LOOP
// =================================================================
void loop() {
  handleOrinCommands();
  handleArduinoIMU();

  if (COMMAND_TIMEOUT_MS > 0 && (currentLeft != 0 || currentRight != 0) &&
      (millis() - lastCommandTime) > COMMAND_TIMEOUT_MS) {
    stopMotors();
  }
}

// =================================================================
//                        MOTOR DRIVE FUNCTION
// =================================================================
void driveSide(
  int in1, int in2, int enA,
  int in3, int in4, int enB,
  int speedVal
) {
  speedVal = constrain(speedVal, -255, 255);

  if (speedVal > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
    analogWrite(enA, speedVal);
    analogWrite(enB, speedVal);
  }
  else if (speedVal < 0) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
    analogWrite(enA, -speedVal);
    analogWrite(enB, -speedVal);
  }
  else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    digitalWrite(in3, LOW); digitalWrite(in4, LOW);
    analogWrite(enA, 0);
    analogWrite(enB, 0);
  }
}

void stopMotors() {
  driveSide(L_IN1, L_IN2, L_ENA, L_IN3, L_IN4, L_ENB, 0);
  driveSide(R_IN1, R_IN2, R_ENA, R_IN3, R_IN4, R_ENB, 0);
  currentLeft = 0;
  currentRight = 0;
}

// =================================================================
//                      COMMAND HANDLER
// =================================================================
// Parses one complete line such as: "LEFT:120 RIGHT:-120 servo1:45 servo2:90"
void processCommandLine(char* line) {
  int target_s1 = servo1.read();
  int target_s2 = servo2.read();
  bool servoCmd = false;

  char* ctx;
  char* token = strtok_r(line, " ", &ctx);

  while (token) {
    char* colon = strchr(token, ':');
    if (colon) {
      *colon = '\0';
      int value = atoi(colon + 1);

      if (strcmp(token, "LEFT") == 0) {
        driveSide(L_IN1, L_IN2, L_ENA, L_IN3, L_IN4, L_ENB, value*LEFT_INVERT);
        currentLeft = value;
        lastCommandTime = millis();
      }
      else if (strcmp(token, "RIGHT") == 0) {
        driveSide(R_IN1, R_IN2, R_ENA, R_IN3, R_IN4, R_ENB, value*RIGHT_INVERT);
        currentRight = value;
        lastCommandTime = millis();
      }
      else if (strcmp(token, "servo1") == 0) {
        target_s1 = value; servoCmd = true;
      }
      else if (strcmp(token, "servo2") == 0) {
        target_s2 = value; servoCmd = true;
      }
    }
    token = strtok_r(NULL, " ", &ctx);
  }

  if (servoCmd) startSmoothMove(target_s1, target_s2);
}

// Non-blocking: collects bytes until '\n', then parses the line. The old
// readBytesUntil() could block for up to 1 s (default Serial timeout) on a partial line,
// stalling the IMU relay and the watchdog, and could act on a truncated command.
void handleOrinCommands() {
  static char line[256];
  static size_t len = 0;
  static bool overflow = false;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (!overflow && len > 0) {
        line[len] = '\0';
        processCommandLine(line);
      }
      len = 0;
      overflow = false;
    } else if (c != '\r') {
      if (len < sizeof(line) - 1) line[len++] = c;
      else overflow = true;      // overlong line: drop all of it, never act on a fragment
    }
  }
}

// =================================================================
//                         IMU RELAY
// =================================================================
void handleArduinoIMU() {
  static uint8_t buffer[sizeof(IMUDataPacket)];
  static uint8_t pos = 0;

  while (Serial1.available()) {
    uint8_t b = Serial1.read();

    if (pos == 0) {
      if (b == '$') buffer[pos++] = b;
    } else {
      buffer[pos++] = b;
      if (pos >= sizeof(IMUDataPacket)) {
        Serial.write(buffer, sizeof(IMUDataPacket));
        pos = 0;
      }
    }
  }
}

// =================================================================
//                   NON-BLOCKING SERVO LOGIC
// =================================================================
void startSmoothMove(int t1, int t2) {
  if (isMovingServos) return;

  targetAngle1 = constrain(t1, SERVO1_MIN_ANGLE, SERVO1_MAX_ANGLE);
  targetAngle2 = constrain(t2, SERVO2_MIN_ANGLE, SERVO2_MAX_ANGLE);

  startAngle1 = servo1.read();
  startAngle2 = servo2.read();

  int maxDelta = max(abs(targetAngle1 - startAngle1),
                     abs(targetAngle2 - startAngle2));

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
