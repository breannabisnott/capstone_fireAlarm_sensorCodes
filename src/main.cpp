#include <Arduino.h>
#include <ArduinoJson.h>

const int flamePin = 34;

void setup() {
  pinMode(flamePin, INPUT);  // Set the flame sensor pin as input
  Serial.begin(9600);
}

void loop(){
  int analogVolts = analogReadMilliVolts(A4);
  long temperature = analogVolts/10.0;
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.print("°C\n");

  if (digitalRead(flamePin) == 0) {
    Serial.println("FIRE DETECTED!");
  } else {
    Serial.println("No Fire detected");
  }
  
  delay(2000);
}