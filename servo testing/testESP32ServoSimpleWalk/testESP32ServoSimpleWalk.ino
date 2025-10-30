/* Sweep


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
 * 
 */
 

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
