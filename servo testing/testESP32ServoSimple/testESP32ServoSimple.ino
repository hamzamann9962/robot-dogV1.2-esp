/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald

 modified for the ESP32 on March 2017
 by John Bennett

 see http://www.arduino.cc/en/Tutorial/Sweep for a description of the original code

 * Different servos require different pulse widths to vary servo angle, but the range is 
 * an approximately 500-2500 microsecond pulse every 20ms (50Hz). In general, hobbyist servos
 * sweep 180 degrees, so the lowest number in the published range for a particular servo
 * represents an angle of 0 degrees, the middle of the range represents 90 degrees, and the top
 * of the range represents 180 degrees. So for example, if the range is 1000us to 2000us,
 * 1000us would equal an angle of 0, 1500us would equal 90 degrees, and 2000us would equal 1800
 * degrees.
 * 
 * Circuit: (using an ESP32 Thing from Sparkfun)
 * Servo motors have three wires: power, ground, and signal. The power wire is typically red,
 * the ground wire is typically black or brown, and the signal wire is typically yellow,
 * orange or white. Since the ESP32 can supply limited current at only 3.3V, and servos draw
 * considerable power, we will connect servo power to the VBat pin of the ESP32 (located
 * near the USB connector). THIS IS ONLY APPROPRIATE FOR SMALL SERVOS. 
 * 
 * We could also connect servo power to a separate external
 * power source (as long as we connect all of the grounds (ESP32, servo, and external power).
 * In this example, we just connect ESP32 ground to servo ground. The servo signal pins
 * connect to any available GPIO pins on the ESP32 (in this example, we use pin 18.
 * 
 * In this example, we assume a Tower Pro MG995 large servo connected to an external power source.
 * The published min and max for this servo is 1000 and 2000, respectively, so the defaults are fine.
 * These values actually drive the servos a little past 0 and 180, so
 * if you are particular, adjust the min and max values to match your needs.
 */

// testESP32ServoSimple_PCA9685.ino
// Drop-in simple sweep test for PCA9685 (ESP32 + 12 servos on channels 0..13 with skips)
// Matches user wiring: RF: 0,1,2 | skip 3 | LF: 4,5,6 | skip 7 | RR: 8,9,10 | LR: 11,12,13

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ====== Config ======
#define PCA_ADDR 0x40
const float SERVO_FREQ_HZ = 50.0f;   // standard analog servo
const uint16_t US_MIN      = 500;    // adjust if needed
const uint16_t US_MAX      = 2500;   // adjust if needed
const uint16_t US_NEUTRAL  = 1500;   // for first move

// Legs/Joints to keep semantics similar (but not strictly used in this simple sweep)
enum Leg   { RF=0, LF=1, RR=2, LR=3 };
enum Joint { SHOULDER=0, KNEE=1, FOOT=2 };

// User wiring mapping per leg [Shoulder, Knee, Foot]
const uint8_t SERVO_CH[4][3] = {
  /* RF */ {  0,  1,  2 }, // skip 3
  /* LF */ {  4,  5,  6 }, // skip 7
  /* RR */ {  8,  9, 10 },
  /* LR */ { 11, 12, 13 }
};

// Direction and offsets (edit if needed)
bool SERVO_INV[4][3] = {
  {false, true,  true },
  {true,  false, true },
  {false, true,  true },
  {true,  false, true }
};
int16_t SERVO_OFFSET[4][3] = {
  {0,0,0},{0,0,0},{0,0,0},{0,0,0}
};

// ====== Globals ======
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA_ADDR);
int pos = 0; // degrees

// Map angle (-90..+90) to microseconds (US_MIN..US_MAX)
uint16_t angleToUs(float deg){
  // apply clamp visible range if you like; we just map directly here
  float t = (deg + 90.0f)/180.0f;      // 0..1
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  return (uint16_t)(US_MIN + t*(US_MAX-US_MIN));
}

void writeJointRawChannel(uint8_t ch, float deg){
  uint16_t us = angleToUs(deg);
  pwm.setPWMus(ch, us);
}

void writeJoint(Leg L, Joint J, float deg){
  deg += SERVO_OFFSET[L][J];
  if (SERVO_INV[L][J]) deg = -deg;
  writeJointRawChannel(SERVO_CH[L][J], deg);
}

void neutralAll(){
  for(int L=0; L<4; ++L){
    for(int J=0; J<3; ++J){
      writeJoint((Leg)L, (Joint)J, 0);
    }
  }
}

void setup(){
  Serial.begin(115200);
  delay(100);
  Wire.begin(21,22); // SDA,SCL
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  delay(10);

  Serial.println(F("PCA9685 Simple Sweep (12 servos)"));
  Serial.println(F("Mapping: RF[0,1,2], LF[4,5,6], RR[8,9,10], LR[11,12,13]"));

  // Pre-position to neutral to avoid sudden jumps later
  neutralAll();
  delay(500);
}

void loop(){
  // Sweep 60..120 deg for all 12 servos (like original example)
  for (pos = 60; pos <= 120; pos += 1) {
    for (int L=0; L<4; ++L){
      for (int J=0; J<3; ++J){
        writeJoint((Leg)L, (Joint)J, pos);
      }
    }
    delay(4);
  }
  for (pos = 120; pos >= 60; pos -= 1) {
    for (int L=0; L<4; ++L){
      for (int J=0; J<3; ++J){
        writeJoint((Leg)L, (Joint)J, pos);
      }
    }
    delay(4);
  }
}
