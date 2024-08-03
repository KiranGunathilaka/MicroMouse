
const int encoderPinA = 18;  // Encoder signal A connected to GPIO 2
const int encoderPinB = 19;  // Encoder signal B connected to GPIO 3

volatile int encoderPos = 0;
volatile int lastEncoded = 0;
volatile int encoderValue = 0;

void readEncoder() {
  int MSB = digitalRead(encoderPinA);
  int LSB = digitalRead(encoderPinB);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderPos++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderPos--;
  }

  lastEncoded = encoded;

  // Print encoder position for testing
  //Serial.print("Encoder Position: ");
  //Serial.println(encoderPos);
}
void setup() {
  // sets the pins as outputs:
  pinMode(27, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), readEncoder, CHANGE);

  ledcAttachChannel(14, 30000, 8, 0);
  ledcWrite(14, 255); 
    Serial.begin(115200);
}
void loop() {
  Serial.print(encoderPos);
  Serial.println("Moving Forward");
  digitalWrite(27, LOW);
  digitalWrite(26, HIGH);

  delay(1000);
  digitalWrite(27, HIGH);
  digitalWrite(26, LOW);
  delay(1000);
}