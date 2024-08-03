#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

//LSM6D3 GYRO 
#define LSM6DS3_ADDRESS 0x6B  // I2C address of LSM6DS3

#define CTRL1_XL 0x10  // Accelerometer control register
#define CTRL2_G  0x11  // Gyroscope control register
#define OUTX_L_G 0x22  // First gyro data register
#define OUTX_L_XL 0x28 // First accel data register

//TOF
VL53L0X sensor1 ,sensor2, sensor3 ;
int a,b,c,d;

//Magnetometer
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void setup() {
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);

  //Gyro Config
  writeByte(CTRL1_XL, 0x54); // set the accel range to +- 4g
  writeByte(CTRL2_G, 0x54); // Set the Gyro range to +- 500 dps
 
  byte ctrl1 = readByte(CTRL1_XL);
  byte ctrl2 = readByte(CTRL2_G);

  Serial.print("CTRL1_XL: 0x");
  Serial.println(ctrl1, HEX);
  Serial.print("CTRL2_G: 0x");
  Serial.println(ctrl2, HEX);


  //ToF address change
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  digitalWrite(4 , LOW);
  digitalWrite(5 , LOW);
  digitalWrite(6 , LOW);

  Serial.begin(115200);
  Wire.begin(20, 21);  // SDA , SCL
  Wire.setClock(100000);
  
  //changing address of 1st tof
  digitalWrite(4, HIGH);
  delay(40);
  sensor1.init(true);
  Serial.println("01");
  delay(40);
  sensor1.setAddress(0x30);

  //changing address of 2nd tof
  digitalWrite(5, HIGH);
  delay(40);
  sensor2.init(true);
  Serial.println("02");
  delay(40);
  sensor2.setAddress(0x31);

  //changing address of 3rd tof
  digitalWrite(6, HIGH);
  delay(40);
  sensor3.init(true);
  Serial.println("03");
  delay(40);
  sensor3.setAddress(0x32);

  Serial.println("ToF addresses set");

  sensor1.startContinuous();
  sensor2.startContinuous();
  sensor3.startContinuous();

  //Magneto
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
  }
}

void loop() {

  a = sensor1.readRangeContinuousMillimeters();
  Serial.print(a);
  Serial.print(" ");

  b = sensor2.readRangeContinuousMillimeters();
  Serial.print(b);
  Serial.print(" ");

  c = sensor3.readRangeContinuousMillimeters();
  Serial.print(c);
  Serial.println(" ");
  Serial.println(" ");


  //Gyro Data Read
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
  Serial.println(" ");



  //read Magno
  sensors_event_t event; 
  mag.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.22;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  Serial.println(" ");



  // //addresses Detect
  // byte error, address;
  // int nDevices;
  // nDevices = 0;
  // for(address = 1; address < 127; address++) {
  //   Wire.beginTransmission(address);
  //   error = Wire.endTransmission();

  //   if (error == 0) {
  //     Serial.print("I2C device found at address 0x");
  //     if (address < 16) 
  //       Serial.print("0");
  //     Serial.print(address, HEX);
  //     Serial.println("  !");

  //     nDevices++;
  //   }
  //   else if (error == 4) {
  //     Serial.print("Unknown error at address 0x");
  //     if (address < 16) 
  //       Serial.print("0");
  //     Serial.println(address, HEX);
  //   }   
  // }

  delay(1000);
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
