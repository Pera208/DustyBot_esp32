#include <Arduino.h>
#include <DHT.h>
#include <DHT_U.h>
#include <WiFi.h>
#include <Pubsubclient.h>

#include "credentials.h"

// -- Config -- //
// Moved to credentials.h

// MQTT broker details
const int mqtt_port = 1884;

// put function declarations here:

// DHT settings
//const int ledPin {7}
const int dhtPin {2};
const char* dhtType {"DHT11"};

uint32_t delayMS;
DHT_Unified dht(dhtPin, DHT11);

// Global variables
WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi();
void reconnect();
void publishSensorData();

void setup() {
  // put your setup code here, to run once:
  //pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  setup_wifi();

  Serial.println("WiFi setup Done!");

  dht.begin();
  Serial.println("DHT11 initialized");

  client.setServer(mqtt_server, mqtt_port);

  sensor_t sensor;
  dht.humidity().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;
}

void loop() {

  // Reconnect if connection is lost.
  if (!client.connected()) {
    reconnect();
  }

  // Call the function to publish sensor data.
  static unsigned long lastPublish = 0;
  unsigned long now = millis();
  // Wait 30 before publishing new data
  if (now - lastPublish > 30000) {
    publishSensorData();
    lastPublish = now;
  }
}

void setup_wifi() {
  WiFi.mode(WIFI_STA);
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  WiFi.setTxPower(WIFI_POWER_8_5dBm); 

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 20000) {
    delay(1000);
    Serial.printf("Attempting to connect, Status: %d\n", WiFi.status());
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {

  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void publishSensorData() {
  // Check if reads failed

  sensors_event_t event;

  dht.temperature().getEvent(&event);
  float temp {event.temperature};
  dht.humidity().getEvent(&event);
  float humidity {event.relative_humidity};

  if (isnan(temp) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Create json payload
  String payload = "{";
  payload += "\"temperature\":" + String(temp) + ",";
  payload += "\"humidity\":" + String(humidity);
  payload += "}";
  
  // Publish to topic home/sensors/dht11
  if (client.publish("home/sensors/dht11", payload.c_str())) {
    Serial.println("Sensor data published");
    Serial.println(payload);
  } else {
    Serial.println("Failed to publish sensor data");
  }

}

// put function definitions here:

/*
int measurePin = 4;
int ledPower = 12;
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;
 
void setup() {
  Serial.begin(9600);
  pinMode(ledPower, OUTPUT);
  //analogReadResolution(12);  // 12-bit ADC resolution
  //analogSetAttenuation(ADC_11db);  // Full 0-3.3V range
}
 
void loop() {
  digitalWrite(ledPower, LOW);
  delayMicroseconds(samplingTime);
  voMeasured = analogRead(measurePin);
  delayMicroseconds(deltaTime);
  digitalWrite(ledPower, HIGH);
  delayMicroseconds(sleepTime);
  calcVoltage = voMeasured * (3.3 / 4095.0);
  dustDensity = 170 * calcVoltage - 0.1;
  Serial.print("ความหนาแน่นของฝุ่นละออง : ");
  Serial.print(dustDensity);
  Serial.println(" ug/m³");
  Serial.print("Calc Voltage : ");
  Serial.println(calcVoltage);
  Serial.print("Vo Measured : ");
  Serial.println(voMeasured);

  delay(1000);

}
*/
