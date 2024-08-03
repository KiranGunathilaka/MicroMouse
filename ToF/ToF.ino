#include <Adafruit_VL53L0X.h>

// Create an instance of the sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

const int SDA_PIN = 5; // Change to your custom SDA pin
const int SCL_PIN = 6; // Change to your custom SCL pin

const int offset = -20;


void setup() {
  Serial.begin(115200);

  // Initialize I2C on custom pins
  Wire.begin(SDA_PIN, SCL_PIN);
  // wait until serial port opens for native USB devices
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
}


void loop() {
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); 
    Serial.println(measure.RangeMilliMeter);
    Serial.println(measure.RangeStatus);
  } else {
    Serial.println(" out of range ");
  }
    
  delay(1000);
}
