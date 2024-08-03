//left A right B
const int encoderPinA1 = 18; 
const int encoderPinA2 = 19;  

const int encoderPinB1 = 15; 
const int encoderPinB2 = 16;  

const int aIn1 = 27;
const int aIn2 = 26;

const int bIn1 = 31;
const int bIn2 = 32;

const int pwmA = 14;
const int pwmB = 13;

volatile int encoderPos1 = 0;
volatile int lastEncoded1 = 0;
volatile int encoderValue1 = 0;

volatile int encoderPos2 = 0;
volatile int lastEncoded2 = 0;
volatile int encoderValue2 = 0;

void readEncoder1() {
  int MSB = digitalRead(encoderPinA1);
  int LSB = digitalRead(encoderPinA2);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded1 << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderPos1++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderPos1--;
  }

  lastEncoded1 = encoded;

}

void readEncoder2() {
  int MSB = digitalRead(encoderPinB1);
  int LSB = digitalRead(encoderPinB2);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded2 << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderPos2++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderPos2--;
  }

  lastEncoded2 = encoded;
}

void setup() {
  // sets the pins as outputs:
  Serial.begin(115200);

  pinMode(aIn1, OUTPUT);
  pinMode(aIn2, OUTPUT);
  pinMode(encoderPinA1, INPUT);
  pinMode(encoderPinA2, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA1), readEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA2), readEncoder1, CHANGE);

  ledcSetup(0, 30000, 8);  // channel 0, 30000 Hz, 8-bit resolution
  ledcAttachPin(pwmA, 0);  // attach aIn1 to channel 0
  ledcWrite(pwmA, 255); 
  

  pinMode(bIn1, OUTPUT);
  pinMode(bIn2, OUTPUT);
  pinMode(encoderPinB1, INPUT);
  pinMode(encoderPinB2, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinB1), readEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB2), readEncoder2, CHANGE);

  ledcSetup(1, 30000, 8);  // channel 1, 30000 Hz, 8-bit resolution
  ledcAttachPin(pwmB, 1);  // attach pwmB to channel 1
  ledcWrite(pwmB, 255); 
}

void loop() {
  Serial.print(encoderPos1);
  Serial.println("Moving Forward");
  digitalWrite(aIn1, LOW);
  digitalWrite(aIn2, HIGH);

  Serial.print(encoderPos2);
  Serial.println("Moving Forward");
  digitalWrite(bIn1, LOW);
  digitalWrite(bIn2, HIGH);
  delay(1000);


  digitalWrite(aIn1, HIGH);
  digitalWrite(aIn2, LOW);

  digitalWrite(bIn1, HIGH);
  digitalWrite(bIn2, LOW);
  delay(1000);
}