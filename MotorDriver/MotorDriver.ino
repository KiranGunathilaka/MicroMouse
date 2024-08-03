#include <WiFi.h>
#include <stdlib.h>
#include <VL53L0X.h>

VL53L0X sensor1, sensor2, sensor3, sensor4;

int right , rightTop, left, leftTop;
int maxRpm = 230;

/////Acc gyro//////
#include <Wire.h>

#define LSM6DS3_ADDRESS 0x6B  // I2C address of LSM6DS3

#define CTRL1_XL 0x10   // Accelerometer control register
#define CTRL2_G 0x11    // Gyroscope control register
#define OUTX_L_G 0x22   // First gyro data register
#define OUTX_L_XL 0x28  // First accel data register

//telnet 192.168.8.194 23


const char* ssid = "Hi Wifi";
const char* password = "Neme9s8i7s";

WiFiServer server(23);  // Telnet port

// Motor A---Right
const int motorA1 = 7;       // IN1
const int motorA2 = 15;       // IN2
const int motorAEnable = 6;  // ENA

// Motor B---Left
const int motorB1 = 17;        // IN3
const int motorB2 = 16;       // IN4
const int motorBEnable = 18;  // ENB

// PWM properties
const int freq = 30000;
const int motorAChannel = 0;
const int motorBChannel = 1;
const int resolution = 8;

const int SDA_PIN = 21;  // Change to your custom SDA pin
const int SCL_PIN = 20;  // Change to your custom SCL pin

const int offset = -30;

void setup() {
  // Start serial communication
  Serial.begin(115200);

  Wire1.begin(48,  47) ;  //SDA ,SCL of accelerometer
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

  // ToF initialization

  pinMode(35, OUTPUT);
  pinMode(36, OUTPUT);
  pinMode(37, OUTPUT);
  pinMode(38, OUTPUT);

  digitalWrite(35, LOW);
  digitalWrite(36, LOW);
  digitalWrite(37, LOW);
  digitalWrite(38, LOW);

  delay(30);

  Wire.begin(21, 20);  //sda , scl
  Serial.begin(115200);


  digitalWrite(35, HIGH);
  delay(30);
  sensor1.init(true);
  delay(30);
  sensor1.setAddress(0x30);


  digitalWrite(36, HIGH);
  delay(30);
  sensor2.init(true);
  delay(30);
  sensor2.setAddress(0x31);

  digitalWrite(37, HIGH);
  delay(30);
  sensor3.init(true);
  delay(30);
  sensor3.setAddress(0x32);

  digitalWrite(38, HIGH);
  delay(30);
  sensor4.init(true);
  delay(30);
  sensor4.setAddress(0x33);


  Serial.println("addresses set");

  sensor1.startContinuous();
  sensor2.startContinuous();
  sensor3.startContinuous();
  sensor4.startContinuous();
}

void loop() {


  WiFiClient client = server.available();
  if (client) {
    while (client.connected()) {
      left = sensor4.readRangeContinuousMillimeters() + offset;
      Serial.print(left);
      Serial.println(" ");

      leftTop = sensor3.readRangeContinuousMillimeters() + offset;
      Serial.print(leftTop);
      Serial.print(" ");

      
      rightTop = sensor2.readRangeContinuousMillimeters() + offset - 30;
      Serial.print(rightTop);
      Serial.print(" ");

      right = sensor1.readRangeContinuousMillimeters() + offset + 10;
      Serial.print(right);
      Serial.print(" ");

      left = sensor4.readRangeContinuousMillimeters() + offset;
      leftTop = sensor3.readRangeContinuousMillimeters() + offset;
      rightTop = sensor2.readRangeContinuousMillimeters() + offset - 30;
      right = sensor1.readRangeContinuousMillimeters() + offset + 10;
    
      forward(&client);

      // if (leftTop < 205 || rightTop < 205){
      //   client.println("Stopping");
      //   stop(&client);
      //   if (left > 205){
      //     client.println("turning right");
      //     turnRight(&client);
          
      //   }else if (right > 205){
      //     client.println("turning left");
      //     turnLeft(&client);
      //   }else {
      //     client.println("turning 180");
      //     turnLeft(&client);
      //     turnLeft(&client);
      //   }
      // }else{
      //   client.println("running Staright");
      //   setMotorA(maxRpm, true);
      //   setMotorB(maxRpm, false);
      // }

      // if (range < 150) {
      //   float gyroZ, theta;
      //   theta = 0;
      //   unsigned long t1;
      //   client.println("150");
      //   while (abs(theta) < 90) {
      //     t1 = millis();
      //     setMotorA(250, true);
      //     setMotorB(250, true);
      //     gyroZ = (float)readWord(OUTX_L_G + 4) * (500.0 / 32768.0);
      //     client.println(theta);
      //     theta += gyroZ * (millis() - t1) / 1000;
      //   }

      // } else if (range < 450) {
      //   setMotorA((range - 150) * 60 / 300 + 200, true);
      //   setMotorB((range - 150) * 60 / 300 + 200, false);
      //   client.println("below 450 above 150");
      // } else {
      //   setMotorA(255, true);
      //   setMotorB(255, false);
      //   client.println("Full speed");
      // }

      client.println("one iteration done");
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

//////Turn Left function///////////
void turnLeft(WiFiClient* client) {
  float gyroZ, theta;
  theta = 0;
  unsigned long t1;
  client->println("turning left");
  while (abs(theta) < 90) {
    t1 = millis();
    setMotorA(maxRpm, true);
    setMotorB(maxRpm, false);
    gyroZ = (float)readWord(OUTX_L_G + 4) * (500.0 / 32768.0);
    client->println(theta);
    theta += gyroZ * (millis() - t1) / 1000;
  }
  setMotorA(0,true);
  setMotorB(0,true);
}

////////////Turn right//////////
void turnRight(WiFiClient* client) {
  float gyroZ, theta;
  theta = 0;
  unsigned long t1;
  client->println("turning right");
  while (abs(theta) < 90) {
    t1 = millis();
    setMotorA(maxRpm, false);
    setMotorB(maxRpm, true);
    gyroZ = (float)readWord(OUTX_L_G + 4) * (500.0 / 32768.0);
    client->println(theta);
    theta += gyroZ * (millis() - t1) / 1000;
  }
  setMotorA(0,true);
  setMotorB(0,true);
}

////////////Forward//////////////
void forward(WiFiClient* client) {
  client->println("accerlerating");
  for (int i = 150; i < maxRpm; i++) {
    setMotorA(i, true);
    setMotorB(i, true);
    client->println(i);
    delay(2);
  }
  while(true){
    left = sensor4.readRangeContinuousMillimeters() + offset;
    leftTop = sensor3.readRangeContinuousMillimeters() + offset;
    rightTop = sensor2.readRangeContinuousMillimeters() + offset - 30;
    right = sensor1.readRangeContinuousMillimeters() + offset + 10;
    client->println(left);
    client->println(leftTop);
    client->println(right);
    client->println(rightTop);
    int L = left + leftTop;
    int R = right + rightTop;
    client->println(L);
    client->println(R);
    setMotorA((L/(L+R))*maxRpm*2, true);
    setMotorB((R/(L+R))*maxRpm*2, true);

  }
}

//////////stop///////////////
void stop(WiFiClient* client) {
  client->println("Stopping");
  for (int i = maxRpm; i > 150; i--) {
    setMotorA(i, true);
    setMotorB(i, true);
    client->println(i);
    delay(10);
  }
  setMotorA(0, true);
  setMotorB(0, true);
}