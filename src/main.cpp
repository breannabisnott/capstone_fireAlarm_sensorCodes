#include <Arduino.h>
#include <ArduinoJson.h>

const int flamePin = 34;
const int gasConcentrationPin = 12;   // Analog pin for MQ-4 sensor
const int gasPin=14; 

void setup() {
  pinMode(flamePin, INPUT);  // Set the flame sensor pin as input
  pinMode(gasPin, INPUT); // Configure D8 pin as a digital input pin
  Serial.begin(9600);
}

void loop(){
  int sensorValue = analogRead(gasConcentrationPin);
  int threshold= digitalRead(gasPin);
  int analogVolts = analogReadMilliVolts(A4);

  long temperature = analogVolts/10.0;
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println("°C");

  if (digitalRead(flamePin) == 0) {
    Serial.println("FIRE DETECTED!");
  } else {
    Serial.println("No Fire detected");
  }
  
  Serial.print("Gas Threshold: ");
  Serial.print(threshold);                      
  Serial.print(", ");
  Serial.print("Gas Concentration : ");
  Serial.print(sensorValue);
  Serial.println(" ppm");

  Serial.println("----------------------------------------");
  delay(2000);
} 

// #include "DFRobot_EOxygenSensor.h"

// #define OXYGEN_I2C_ADDRESS E_OXYGEN_ADDRESS_3
// DFRobot_EOxygenSensor_I2C oxygen(&Wire, OXYGEN_I2C_ADDRESS);

// void setup()
// {
//   Serial.begin(9600);
//   while(!Serial);
//   while(!oxygen.begin()){
//     Serial.println("NO Deivces !");
//     delay(1000);
//   } Serial.println("Device connected successfully !");
// }

// void loop() 
// {
//   Serial.print("oxygen concetnration is "); 
//   Serial.print(oxygen.readOxygenConcentration());
//   Serial.println("% VOL");
//   delay(1000);
// }