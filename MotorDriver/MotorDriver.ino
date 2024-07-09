
#include <Adafruit_VL53L0X.h>
#include <WiFi.h>
#include <stdlib.h>

/////Acc gyro//////
#include <Wire.h>

#define LSM6DS3_ADDRESS 0x6B  // I2C address of LSM6DS3

#define CTRL1_XL 0x10  // Accelerometer control register
#define CTRL2_G  0x11  // Gyroscope control register
#define OUTX_L_G 0x22  // First gyro data register
#define OUTX_L_XL 0x28 // First accel data register

//telnet 192.168.8.194 23


const char* ssid = "Hi Wifi";
const char* password = "Neme9s8i7s";

WiFiServer server(23);  // Telnet port

// Motor A
const int motorA1 = 5;       // IN1
const int motorA2 = 6;       // IN2
const int motorAEnable = 4;  // ENA

// Motor B
const int motorB1 = 7;       // IN3
const int motorB2 = 8;       // IN4
const int motorBEnable = 9;  // ENB

// PWM properties
const int freq = 30000;
const int motorAChannel = 0;
const int motorBChannel = 1;
const int resolution = 8;

// Create an instance of the sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

const int SDA_PIN = 20;  // Change to your custom SDA pin
const int SCL_PIN = 21;  // Change to your custom SCL pin

const int offset = -20;

void setup() {
  // Start serial communication
  Serial.begin(115200);

  Wire1.begin(35,  36) ;  //SDA ,SCL of accelerometer
  writeByte(CTRL1_XL, 0x54); // set the accel range to +- 4g
  writeByte(CTRL2_G, 0x54); // Set the Gyro range to +- 500 dps
 
  byte ctrl1 = readByte(CTRL1_XL);
  byte ctrl2 = readByte(CTRL2_G);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting to WiFi...");
    delay(1000);
    
  }
  Serial.println("Connected to WiFi");
  // Set all the motor control pins to outputs

  server.begin();

  Wire.begin(SDA_PIN, SCL_PIN);
  // wait until serial port opens for native USB devices
  while (!Serial) {
    delay(1);
  }

  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1)
      ;
  }

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorAEnable, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorBEnable, OUTPUT);

  // Configure PWM functionalities
  ledcSetup(motorAChannel, freq, resolution);
  ledcSetup(motorBChannel, freq, resolution);

  // Attach the channels to the GPIO pins that you wish to control
  ledcAttachPin(motorAEnable, motorAChannel);
  ledcAttachPin(motorBEnable, motorBChannel);
}

void loop() {

  VL53L0X_RangingMeasurementData_t measure;

  WiFiClient client = server.available();
  if (client) {
    while (client.connected()) {
      lox.rangingTest(&measure, false);  // pass in 'true' to get debug data printout!

      int range = measure.RangeMilliMeter + offset;

      if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        client.print("Distance (mm): ");
        client.println(range);
      } else {
        client.println(" out of range ");
      }

      if (range < 150) {
        float gyroZ, theta;
        theta = 0;
        unsigned long t1;
        client.println("150");
        while (abs(theta) < 90){
            t1= millis();
            setMotorA(250, true);
            setMotorB(250, true);
            gyroZ = (float)readWord(OUTX_L_G + 4) * (500.0/32768.0);
            client.println(theta);
            theta += gyroZ*( millis()-t1 )/1000;
            
          }

      } else if (range < 450) {
        setMotorA((range-150)*60/300+200, true);
        setMotorB((range-150)*60/300+200, false);
        client.println("below 450 above 150");
      } else {
        setMotorA(255, true);
        setMotorB(255, false);
        client.println("Full speed");
      }

      client.println("Out");
    }
  }
}


////////////////Motor Controlling////////////////////////
void setMotorA(int speed, bool forward) {
  ledcWrite(motorAChannel, speed);
  digitalWrite(motorA1, forward ? HIGH : LOW);
  digitalWrite(motorA2, forward ? LOW : HIGH);
}

void setMotorB(int speed, bool forward) {
  ledcWrite(motorBChannel, speed);
  digitalWrite(motorB1, forward ? HIGH : LOW);
  digitalWrite(motorB2, forward ? LOW : HIGH);
}

////////////////Accalelerometer and gyro readings//////////////
void writeByte(uint8_t reg, uint8_t value) {
  Wire1.beginTransmission(LSM6DS3_ADDRESS);
  Wire1.write(reg);
  Wire1.write(value);
  Wire1.endTransmission();
}

int16_t readWord(uint8_t reg) {
  Wire1.beginTransmission(LSM6DS3_ADDRESS);
  Wire1.write(reg);
  Wire1.endTransmission(false);
  Wire1.requestFrom(LSM6DS3_ADDRESS, 2);
  return (Wire1.read() | Wire1.read() << 8);
}

int16_t readByte(uint8_t reg) {
  Wire1.beginTransmission(LSM6DS3_ADDRESS);
  Wire1.write(reg);
  Wire1.endTransmission(false);
  Wire1.requestFrom(LSM6DS3_ADDRESS, 1);
  return Wire1.read();
}