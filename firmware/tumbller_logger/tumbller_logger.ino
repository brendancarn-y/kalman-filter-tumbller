// tumbller_logger.ino
// Reads the MPU6050 at a fixed 100 Hz and prints one CSV line per sample:
//   t_us,ax,ay,az,gx,gy,gz
// Motors are disabled. No IMU library needed: talks to the chip directly over I2C.

#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;          // MPU6050 I2C address
const uint8_t STBY_PIN = 8;             // TB6612 motor driver enable (pin table: D8)
const unsigned long PERIOD_US = 10000;  // 10 ms between samples = 100 Hz

void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void setup() {
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, LOW);          // keep the motors off while logging

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);                // fast I2C

  writeReg(0x6B, 0x01);                 // wake chip, clock from gyro X (more stable)
  writeReg(0x1B, 0x00);                 // gyro range +-250 deg/s  -> 131 counts per deg/s
  writeReg(0x1C, 0x00);                 // accel range +-2 g       -> 16384 counts per g
  delay(100);

  Serial.println("t_us,ax,ay,az,gx,gy,gz");
}

void loop() {
  static unsigned long next = micros();
  while ((long)(micros() - next) < 0) { }   // wait for the next 10 ms tick
  next += PERIOD_US;

  unsigned long t = micros();               // timestamp of this sample

  // Ask for 14 bytes starting at register 0x3B:
  // accel X,Y,Z, temperature, gyro X,Y,Z (each 2 bytes, high byte first)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) {
    Serial.println("#I2C_ERROR");
    return;
  }

  int16_t v[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
  }

  Serial.print(t);    Serial.print(',');
  Serial.print(v[0]); Serial.print(',');   // ax
  Serial.print(v[1]); Serial.print(',');   // ay
  Serial.print(v[2]); Serial.print(',');   // az
  Serial.print(v[4]); Serial.print(',');   // gx  (v[3] is temperature, skipped)
  Serial.print(v[5]); Serial.print(',');   // gy
  Serial.println(v[6]);                    // gz
}
