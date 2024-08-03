#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor1 ,sensor2, sensor3 ;

void setup() {
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);

  Serial.begin(115200);
  Wire.begin(20, 21);  // SDA , SCL
  Wire.setClock(100000);

  Serial.println("\nI2C Scanner");
}

void loop() {

  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.println(address, HEX);
    }   
  }
  
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);  // Wait 5 seconds for next scan
}