/************************************************************
 * MPU6050 DMP Test — ESP32 (Wire on SDA=21, SCL=22)
 * Edited for: hamza mannai — adapting for robot dog on ESP32 + PCA9685
 * Source base: Jeff Rowberg's I2Cdevlib (MPU6050_6Axis_MotionApps612)
 *
 * What this does
 * - Initializes I2C @ 400kHz on pins 21(SDA)/22(SCL)
 * - Initializes MPU6050, loads DMP v6.12 firmware
 * - Calibrates accel/gyro briefly, enables DMP
 * - Prints Yaw, Pitch, Roll (degrees) each time a FIFO packet arrives
 * - No external INT pin required (polls FIFO with dmpGetCurrentFIFOPacket)
 *
 * Notes (hamza mannai)
 * - Power MPU6050 at 3.3V on ESP32 (or 5V module with on-board reg/level shifting)
 * - Ensure common GND with ESP32
 * - If AD0 is HIGH, address becomes 0x69 (change MPU_ADDR below)
 * - If you have pull-ups on the breakout, that's fine; avoid double 5V pull-ups
 ************************************************************/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include <Wire.h>
#endif

// ====== User config (hamza mannai) ======
#define SDA_PIN     21
#define SCL_PIN     22
#define I2C_HZ      400000
#define MPU_ADDR    0x68   // set to 0x69 if AD0 pulled high
#define LED_PIN     2      // ESP32 onboard LED is often GPIO2 (change if needed)

// ====== Output format ======
#define OUTPUT_READABLE_YAWPITCHROLL

// ====== MPU wrapper ======
class MPU6050_DMP_ESP32 : public MPU6050 {
public:
  bool     dmpReady   = false;
  uint8_t  devStatus  = 0;
  uint16_t packetSize = 0;
  uint8_t  fifoBuffer[64];

  Quaternion q;
  VectorFloat gravity;
  float ypr[3];

  MPU6050_DMP_ESP32(uint8_t addr) : MPU6050(addr) {}

  void begin() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_HZ);
  #endif

    pinMode(LED_PIN, OUTPUT);

    Serial.println(F("[hamza mannai] Initializing MPU6050..."));
    initialize();
    Serial.print(F("[hamza mannai] Test connection: "));
    Serial.println(testConnection() ? F("OK") : F("FAILED"));

    // Load DMP
    Serial.println(F("[hamza mannai] Loading DMP firmware..."));
    devStatus = dmpInitialize();

    // Optional: set your own offsets if known (values depend on each unit)
    setXGyroOffset(51);
    setYGyroOffset(8);
    setZGyroOffset(21);
    setXAccelOffset(1150);
    setYAccelOffset(-50);
    setZAccelOffset(1060);

    if (devStatus == 0) {
      // Quick calibration (hold device still for ~2s)
      Serial.println(F("[hamza mannai] Calibrating... keep the sensor still."));
      CalibrateAccel(6);
      CalibrateGyro(6);
      PrintActiveOffsets();

      setDMPEnabled(true);
      dmpReady   = true;
      packetSize = dmpGetFIFOPacketSize();
      Serial.print(F("[hamza mannai] DMP enabled. Packet size: "));
      Serial.println(packetSize);
    } else {
      Serial.print(F("[hamza mannai] DMP init failed, code "));
      Serial.println(devStatus);
    }
  }

  bool readYPR(float out_ypr[3]) {
    if (!dmpReady) return false;
    if (dmpGetCurrentFIFOPacket(fifoBuffer)) {
      dmpGetQuaternion(&q, fifoBuffer);
      dmpGetGravity(&gravity, &q);
      dmpGetYawPitchRoll(ypr, &q, &gravity);
      out_ypr[0] = ypr[0] * 180.0f / M_PI;
      out_ypr[1] = ypr[1] * 180.0f / M_PI;
      out_ypr[2] = ypr[2] * 180.0f / M_PI;
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      return true;
    }
    return false;
  }
};

MPU6050_DMP_ESP32 mpu(MPU_ADDR);

// =========== Arduino ===========
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println(F("\n[hamza mannai] MPU6050 DMP test starting..."));
  mpu.begin();
}

void loop() {
  float ypr[3];
  if (mpu.readYPR(ypr)) {
    Serial.print(F("[hamza mannai] ypr: "));
    Serial.print(ypr[0], 2); Serial.print('\t');
    Serial.print(ypr[1], 2); Serial.print('\t');
    Serial.println(ypr[2], 2);
  }
}
