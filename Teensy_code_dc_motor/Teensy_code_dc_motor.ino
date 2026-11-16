#include <Servo.h>
#include <TeensyTimerTool.h>
using namespace TeensyTimerTool;

// =================================================================
//                          HARDWARE SETUP
// =================================================================

// ----- SERVOS -----
Servo servo1;
Servo servo2;

// ----- LEFT DRIVER -----
// Front-Left motor
#define L_IN1 14
#define L_IN2 15
#define L_ENA 9   // PWM
#define LEFT_INVERT  -1


// Back-Left motor
#define L_IN3 17
#define L_IN4 16
#define L_ENB 10  // PWM


// ----- RIGHT DRIVER -----
// Front-Right motor
#define R_IN1 18
#define R_IN2 19
#define R_ENA 2   // PWM
#define RIGHT_INVERT  1
// Back-Right motor
#define R_IN3 20
#define R_IN4 21
#define R_ENB 3  // PWM

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
  // if (millis() - lastCommandTime > 500){
  //   stopMotors();
  // }
  handleOrinCommands();
  handleArduinoIMU();
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

// =================================================================
//                      COMMAND HANDLER
// =================================================================
void handleOrinCommands() {
  static char command_buffer[256];

  if (Serial.available()) {
    int n = Serial.readBytesUntil('\n', command_buffer, sizeof(command_buffer) - 1);
    command_buffer[n] = '\0';

    int target_s1 = servo1.read();
    int target_s2 = servo2.read();
    bool servoCmd = false;

    char* ctx;
    char* token = strtok_r(command_buffer, " ", &ctx);

    while (token) {
      char* colon = strchr(token, ':');
      if (colon) {
        *colon = '\0';
        int value = atoi(colon + 1);

        if (strcmp(token, "LEFT") == 0) {
          driveSide(L_IN1, L_IN2, L_ENA, L_IN3, L_IN4, L_ENB, value*LEFT_INVERT);
        }
        else if (strcmp(token, "RIGHT") == 0) {
          driveSide(R_IN1, R_IN2, R_ENA, R_IN3, R_IN4, R_ENB, value*RIGHT_INVERT);
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
