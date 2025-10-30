#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ===== PCA9685 =====
#define PCA_ADDR 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA_ADDR);

// 50 Hz for hobby servos
const float SERVO_FREQ_HZ = 50.0f;
const uint16_t US_MIN = 500;   // adjust if your servos need different range
const uint16_t US_MAX = 2500;  // typical 500–2500us
const uint16_t US_NEUTRAL = 1500;

// ===== Legs / Joints =====
enum Leg   { RF=0, LF=1, RR=2, LR=3 };
enum Joint { SHOULDER=0, KNEE=1, FOOT=2 };

// Your wiring: [Shoulder, Knee, Foot] per leg
const uint8_t SERVO_CH[4][3] = {
  /* RF */ { 0, 1,  2 }, 
  /* LF */ { 4, 5,  6 }, 
  /* RR */ { 8, 9, 10 },
  /* LR */ {11,12, 13 }
};

// Direction fixes (true = invert)
bool SERVO_INV[4][3] = {
  {false, true,  true },   // RF
  {true,  false, true },   // LF
  {false, true,  true },   // RR
  {true,  false, true }    // LR
};

// Zero-angle mechanical offsets (in degrees) – tune these after first test
int16_t SERVO_OFFSET[4][3] = {
  {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}
};

// Software joint safety limits (deg) — set to your linkage limits
const int16_t JOINT_MIN[3] = { -25, -45, -70 };
const int16_t JOINT_MAX[3] = {  25,  70,  20 };

// ===== Helpers =====
uint16_t usToTicks(uint16_t us) {
  // PCA9685: 12-bit (4096 ticks), 25MHz clock, prescale set by setPWMFreq
  // Adafruit lib handles the prescale; we use setPWMus helper
  return us; // not used directly because we'll call setPWMus
}

uint16_t angleToUs(float deg) {
  // Map -90..+90 to US_MIN..US_MAX, then clamp to limits
  float t = (deg + 90.0f) / 180.0f;     // 0..1
  uint16_t us = (uint16_t)(US_MIN + t * (US_MAX - US_MIN));
  return us;
}

float clampDeg(float deg, Joint j) {
  if (deg < JOINT_MIN[j]) deg = JOINT_MIN[j];
  if (deg > JOINT_MAX[j]) deg = JOINT_MAX[j];
  return deg;
}

// Write joint angle (deg, where 0 is your logical neutral before offsets)
void writeJoint(Leg leg, Joint joint, float deg, bool smooth = true) {
  // apply offset & limits
  deg += SERVO_OFFSET[leg][joint];
  deg = clampDeg(deg, joint);
  if (SERVO_INV[leg][joint]) deg = -deg;

  uint16_t targetUs = angleToUs(deg);

  // Smooth ramp to reduce jerk
  static uint16_t lastUs[4][3];
  uint16_t cur = lastUs[leg][joint] ? lastUs[leg][joint] : US_NEUTRAL;

  int step = (targetUs > cur) ? 4 : -4;    // tune for smoothness/speed
  if (!smooth) step = (targetUs > cur) ? 9999 : -9999;

  for (int u = cur; (step > 0) ? (u <= (int)targetUs) : (u >= (int)targetUs); u += step) {
    pwm.setPWMus(SERVO_CH[leg][joint], u);
    delayMicroseconds(1500);
  }
  lastUs[leg][joint] = targetUs;
}

void neutralAll(bool smooth = true) {
  for (int L=0; L<4; ++L)
    for (int J=0; J<3; ++J)
      writeJoint((Leg)L, (Joint)J, 0, smooth);
}

void testSequenceOneByOne() {
  Serial.println(F("Testing each joint: -15 -> 0 -> +15 -> 0 (deg)"));
  for (int L=0; L<4; ++L) {
    for (int J=0; J<3; ++J) {
      Serial.printf("Leg %d, Joint %d\n", L, J);
      writeJoint((Leg)L, (Joint)J, -15); delay(400);
      writeJoint((Leg)L, (Joint)J,   0); delay(300);
      writeJoint((Leg)L, (Joint)J, +15); delay(400);
      writeJoint((Leg)L, (Joint)J,   0); delay(300);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(21,22); // SDA,SCL (change if you wired differently)
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  delay(10);

  Serial.println(F("\nPCA9685 Quadruped Test — mapping RF[0,1,2], LF[4,5,6], RR[8,9,10], LR[11,12,13]"));
  Serial.println(F("Commands:\n  n = neutral all\n  t = test sequence\n  m L J A = move (Leg, Joint, AngleDeg)\n"));
  neutralAll(false);
}

void loop() {
  // simple serial command interface
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'n') {
      Serial.println(F("Neutral all"));
      neutralAll();
    } else if (c == 't') {
      testSequenceOneByOne();
    } else if (c == 'm') {
      // expects: m <leg> <joint> <angle>
      while (Serial.available() == 0) ; // wait for space
      int leg = Serial.parseInt();
      while (Serial.available() == 0) ;
      int joint = Serial.parseInt();
      while (Serial.available() == 0) ;
      int angle = Serial.parseInt();
      Serial.printf("Move L=%d J=%d to %d deg\n", leg, joint, angle);
      writeJoint((Leg)leg, (Joint)joint, (float)angle);
    }
  }
}
