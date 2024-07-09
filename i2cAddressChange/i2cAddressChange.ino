#include <Wire.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // Wait for serial console to open

  // Initialize I2C on pins 21 (SCL) and 20 (SDA)
  Wire.begin(20, 21);

  Serial.println("VL53L0X test");
  if (!lox.begin(0x31, &Wire)) {  // 0x29 is the default address
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }else{
    Serial.println("At i2c add 0x31");
  }

  // Change the I2C address
  if (!lox.setAddress(0x33)) {
    Serial.println(F("Failed to set new address"));
    while(1);
  }else{
    Serial.println(F("Address changed to 0x33"));
  }

  

  // Now initialize with the new address
  if (!lox.begin(0x31, &Wire)) {
    Serial.println(F("Failed to boot VL53L0X at new address"));
    while(1);
  }

  Serial.println(F("VL53L0X initialized with new address"));
}

void loop() {
  // Your main code here
}