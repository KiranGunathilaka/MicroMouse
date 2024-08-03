#include <Arduino.h>

const int ENCODER_A_PIN = 4;  // Change to your actual pin
const int ENCODER_B_PIN = 5;  // Change to your actual pin

volatile long encoderCount = 0;

void IRAM_ATTR encoderISR() {
  if (digitalRead(ENCODER_B_PIN) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
  Serial.println(encoderCount);
}

void setup() {
  Serial.begin(115200);
  
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, RISING);
}

void loop() {
  Serial.println(encoderCount);
  delay(100);
}