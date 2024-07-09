#include <Wire.h>

#define LSM6DS3_ADDRESS 0x6B  // I2C address of LSM6DS3

#define CTRL1_XL 0x10  // Accelerometer control register
#define CTRL2_G  0x11  // Gyroscope control register
#define OUTX_L_G 0x22  // First gyro data register
#define OUTX_L_XL 0x28 // First accel data register

void setup() {
  Wire.begin(35,  36) ;  //SDA ,SCL
  Serial.begin(115200);


  writeByte(CTRL1_XL, 0x54); // set the accel range to +- 4g
  writeByte(CTRL2_G, 0x54); // Set the Gyro range to +- 500 dps
 
  byte ctrl1 = readByte(CTRL1_XL);
  byte ctrl2 = readByte(CTRL2_G);

  Serial.print("CTRL1_XL: 0x");
  Serial.println(ctrl1, HEX);
  Serial.print("CTRL2_G: 0x");
  Serial.println(ctrl2, HEX);

}

void loop() {
  float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
  
  // Read accelerometer data
  accelX = (float)readWord(OUTX_L_XL) * (4.0/32768.0);
  accelY = (float)readWord(OUTX_L_XL + 2) * (4.0/32768.0);
  accelZ = (float)readWord(OUTX_L_XL + 4) * (4.0/32768.0);
  
  // Read gyroscope data
  gyroX = (float)readWord(OUTX_L_G) * (500.0/32768.0);
  gyroY = (float)readWord(OUTX_L_G + 2) * (500.0/32768.0);
  gyroZ = (float)readWord(OUTX_L_G + 4) * (500.0/32768.0);
  
  // Print data
  Serial.print("Accel X: "); Serial.print(accelX);
  Serial.print(" Y: "); Serial.print(accelY);
  Serial.print(" Z: "); Serial.print(accelZ);
  Serial.print(" | Gyro X: "); Serial.print(gyroX);
  Serial.print(" Y: "); Serial.print(gyroY);
  Serial.print(" Z: "); Serial.println(gyroZ);
  
  delay(100);
}


void writeByte(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(LSM6DS3_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

int16_t readWord(uint8_t reg) {
  Wire.beginTransmission(LSM6DS3_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DS3_ADDRESS, 2);
  return (Wire.read() | Wire.read() << 8);
}

int16_t readByte(uint8_t reg) {
  Wire.beginTransmission(LSM6DS3_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DS3_ADDRESS, 1);
  return Wire.read();
}