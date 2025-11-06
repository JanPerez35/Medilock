#include <Arduino.h>

const int ledPin = 2; // Built-in LED for most ESP32 boards

void setup()
{
  pinMode(ledPin, OUTPUT);
}

void loop()
{
  digitalWrite(ledPin, HIGH); // Turn LED on
  delay(1000);
  digitalWrite(ledPin, LOW); // Turn LED off
  delay(1000);
}
