#include <Arduino.h>
#include <ArduinoJson.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include "DFRobot_OxygenSensor.h"

//receiver mac address
uint8_t broadcastAddress[] = {0x88, 0x13, 0xBF, 0x24, 0xFA, 0xAC};

#define flamePin 34  //digital flame status
#define flameLevelPin 35 //analog flame status
#define gasConcentrationPin 32   // Analog pin for MQ-9 sensor
#define gasPin 13  //digital gas status
#define tempPin 33 // temperature pin
#define DHT22_PIN  19 // ESP32 pin GPIO21 connected to DHT22 sensor
#define Oxygen_IICAddress ADDRESS_3
#define COLLECT_NUMBER  10             // collect number, the collection range is 1-100.
DFRobot_OxygenSensor oxygen;           // collect number, the collection range is 1-100.
//#define SENSOR_ADDRESS 0x73

#define uS_TO_S_FACTOR 1000000  // Conversion factor for microseconds to seconds
#define SLEEP_SECONDS 60        // Deep sleep duration (e.g., 10 seconds)

void read_sensor_data();

bool fire_status;

OneWire oneWire(tempPin);
DallasTemperature sensors(&oneWire);
DHT dht22(DHT22_PIN, DHT22);

uint8_t calibrationState = 0;

typedef struct sensor_data {
  char device_id[50];
  float temperature;
  bool fire_status; //digital flame
  float fire_level; //analog flame
  bool threshold;  //gas thresh
  float sensorValue; // gas conc
  float oxygen_level; // o2 conc
  float humidity;
} sensor_data;

sensor_data sensorData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  pinMode(flamePin, INPUT);  // Set the flame sensor pin as input
  pinMode(gasPin, INPUT); // Configure D8 pin as a digital input pin
  pinMode(gasConcentrationPin, INPUT);
  Serial.begin(9600);
  
  while(!oxygen.begin(Oxygen_IICAddress)){
    Serial.println("I2c device number error !");
    delay(1000);
  }
  Serial.println("I2c connect success !");
  Wire.begin();
  
  sensors.begin();
  dht22.begin(); // initialize the DHT22 sensor

  delay(1000);

  analogSetAttenuation(ADC_11db);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Set the Wi-Fi channel (1-14, usually 1-11 in most regions)
  int channel = 6; // Choose a channel with least interference
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  read_sensor_data();

  sleep:
  Serial.flush();  // Ensure all serial data is sent
  delay(100);      // Small delay to ensure completion
  
  // Proper deep sleep initiation
  esp_deep_sleep(SLEEP_SECONDS * uS_TO_S_FACTOR);
  
  // This next line should never be reached
  Serial.println("Failed to enter deep sleep!");
  
}

void read_sensor_data(){
  int sensorValue = analogRead(gasConcentrationPin);
  int threshold= digitalRead(gasPin);
  sensorData.threshold = threshold;     //digital gas reading
  sensorData.sensorValue =  sensorValue;    //analog gas reading

  sensors.requestTemperatures(); 
  float temperature = sensors.getTempCByIndex(0);

  sensorData.humidity  = dht22.readHumidity();    //humidity
  sensorData.temperature = dht22.readTemperature();   //temperature

  sensorData.oxygen_level = oxygen.getOxygenData(COLLECT_NUMBER);
  sensorData.fire_status = digitalRead(flamePin);
  sensorData.fire_level = analogRead(flameLevelPin);

  // read temperature in Fahrenheit
  float tempF = dht22.readTemperature(true);

  strcpy(sensorData.device_id, "AC:15:18:D7:B0:80"); 
  Serial.print("Device ID: ");
  Serial.println(sensorData.device_id);

  Serial.print("Temperature: ");
  Serial.print(sensorData.temperature);
  Serial.print("°C , ");

  Serial.print("Humidity: ");
  Serial.print(sensorData.humidity);
  Serial.println("%");
  
  
  Serial.print("Flame Detection : ");
  Serial.print(sensorData.fire_status);
  Serial.print(", ");
  
  Serial.print("Flame Distance : ");
  Serial.println(sensorData.fire_level);

  Serial.print("Gas Threshold: ");
  Serial.print(sensorData.threshold);                      
  Serial.print(", ");
  Serial.print("Gas Concentration : ");
  Serial.print(sensorData.sensorValue);
  Serial.println(" ppm");
  
  Serial.print("O2 Concentration : ");
  Serial.print(sensorData.oxygen_level);
  Serial.println("%");



  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sensorData, sizeof(sensorData));
   
  if (result == ESP_OK) {
    Serial.println("Data sent with success to CMS");
  }
  else {
    Serial.println("Error sending the data");
  }

  Serial.println("----------------------------------------");
}

void loop(){
  
} 