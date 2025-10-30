// testESP32ServoSimpleWalk_PCA9685.ino
// Ported to PCA9685 for ESP32 + 12 servos using user's channel layout.
// Keeps original arrays and serial controls, but writes via PCA9685.
//
// Wiring per leg (RF, LF, RR, LR):
//   roll  -> ch 0,4,8,11
//   pitch -> ch 2,6,10,13
//   knee  -> ch 1,5,9,12
//
// NOTE: If any joint moves in the wrong direction, flip SERVO_INV[] for that index.
//       If neutral is off, adjust SERVO_OFFSET[] for that index (degrees).

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm(0x40);

// Servo timing (adjust if needed)
const float SERVO_FREQ_HZ = 50.0f;
const uint16_t US_MIN = 500;
const uint16_t US_MAX = 2500;
const uint16_t US_NEUTRAL = 1500;

// Index mapping: sketch index 0..11 -> PCA9685 channel
// Indices 0..3: roll (RF, LF, RR, LR)
// Indices 4..7: pitch (RF, LF, RR, LR)
// Indices 8..11: knee (RF, LF, RR, LR)
const uint8_t IDX_TO_CH[12] = {
  /* roll  */  0, 4, 8, 11,
  /* pitch */  2, 6,10, 13,
  /* knee  */  1, 5, 9, 12
};

// Per-index invert and zero-offset (deg)
bool   SERVO_INV[12]    = { false,false,false,false,  false,false,false,false,  false,false,false,false };
int8_t SERVO_OFFSET[12] = {  0,   0,   0,   0,        0,   0,   0,   0,        0,   0,   0,   0 };

// Angle -> microseconds
uint16_t angleToUs(float deg){
  // Map -90..+90 to US_MIN..US_MAX
  float t = (deg + 90.0f)/180.0f;
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  return (uint16_t)(US_MIN + t*(US_MAX-US_MIN));
}

void writeIdx(uint8_t idx, float deg){
  deg += SERVO_OFFSET[idx];
  if (SERVO_INV[idx]) deg = -deg;
  uint16_t us = angleToUs(deg);
  pwm.setPWMus(IDX_TO_CH[idx], us);
}

void offIdx(uint8_t idx){
  // turn PWM off
  pwm.setPWM(IDX_TO_CH[idx], 0, 0);
}

// ===== Original arrays (copied) =====
int zero[] = {
  90,
  90,
  90,
  90,

  90,
  90,
  90,
  90,

  90,
  90,
  90,
  90,
};

int shift[] = {
  0,
  0,
  0,
  0,
  30,
  -30,
  60,
  -60,
  50,
  -50,
  -50,
  50,
};

int rest[] = {
  0,
  0,
  0,
  0,
  45,
  -45,
  -45,
  45,
  45,
  -45,
  -45,
  45,
};

int stand[] = {
  0,
  0,
  0,
  0,
  30,
  -30,
  -30,
  30,
  -15,
  15,
  15,
  -15,
};

char token = 'c';

// ===== Original behaviors, adapted to writeIdx() =====
void testPosture() {
  for (byte s = 0; s < 12; s++)
    writeIdx(s, zero[s] + shift[s]);
  delay(2000);
  for (byte s = 0; s < 12; s++)
    writeIdx(s, zero[s] + shift[s] + stand[s]);
  delay(2000);
  for (byte s = 0; s < 12; s++)
    writeIdx(s, zero[s] + shift[s] + rest[s]);
}

void testGait() {
  const int amplitude = 10;
  for (int pos = -amplitude; pos <= +amplitude; pos += 1) {
    for (byte s = 0; s < 12; s++)
      writeIdx(s, pos * (s % 4 / 2 ? 1 : -1) * (s > 7 ? 1 : 1)
                   + zero[s] + shift[s] + stand[s]);
    delay(10);
  }
  for (int pos = +amplitude; pos >= -amplitude; pos -= 1) {
    for (byte s = 0; s < 12; s++)
      writeIdx(s, pos * (s % 4 / 2 ? 1 : -1) * (s > 7 ? 1 : 1)
                   + zero[s] + shift[s] + stand[s]);
    delay(10);
  }
}

void setup(){
  Serial.begin(115200);
  delay(100);
  Wire.begin(21,22); // change if needed
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  delay(10);

  // Pre-position to zero+shift
  for (byte s = 0; s < 12; s++)
    writeIdx(s, zero[s] + shift[s]);
}

void loop(){
  if (Serial.available()) {
    token = Serial.read();
    Serial.println(token);
    if (token == 'z')       // go to zero[]
      for (byte s = 0; s < 12; s++)
        writeIdx(s, zero[s]);
    else if (token == 'c')  // go to zero[] + shift[]
      for (byte s = 0; s < 12; s++)
        writeIdx(s, zero[s] + shift[s]);
    else if (token == 'd')  // disable PWM
      for (byte s = 0; s < 12; s++)
        offIdx(s);
    else if (token == 't')  // go to stand[]
      for (byte s = 0; s < 12; s++)
        writeIdx(s, stand[s]);
  }
  if (token == 'k')
    testGait();
}
