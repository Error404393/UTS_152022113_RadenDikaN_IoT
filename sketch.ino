#include <WiFi.h>
#include "PubSubClient.h"
#include "DHTesp.h"

WiFiClient espClient;
PubSubClient mqtt(espClient);

const char* wifiSSID = "Wokwi-GUEST";
const char* wifiPassword = "";

const char* mqttBroker = "broker.hivemq.com";
const int mqttPortNumber = 1883;

const int DHTSensorPin = 32;
const int relayControlPin = 13;

#define RED_LED 18
#define YELLOW_LED 5
#define GREEN_LED 17
#define ALERT_BUZZER 8

DHTesp dht;

void mqttReconnect() {
  Serial.println("Attempting to connect to MQTT...");
  while (!mqtt.connected()) {
    Serial.println("Trying to reconnect to MQTT...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);

    if (mqtt.connect(clientId.c_str())) {
      Serial.println("MQTT connected successfully.");
    }
  }
}

void setupMQTTClient() {
  mqtt.setServer(mqttBroker, mqttPortNumber);
  mqtt.setCallback(mqttCallback);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT Message Received: ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing ESP32...");
  
  dht.setup(DHTSensorPin, DHTesp::DHT22);
  pinMode(relayControlPin, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(ALERT_BUZZER, OUTPUT);

  WiFi.begin(wifiSSID, wifiPassword);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected.");

  setupMQTTClient();
}

void loop() {
  if (!mqtt.connected()) {
    mqttReconnect();
  }
  mqtt.loop();

  TempAndHumidity reading = dht.getTempAndHumidity();
  float temp = reading.temperature;
  float hum = reading.humidity;

  bool redLEDStatus = false;
  bool yellowLEDStatus = false;
  bool greenLEDStatus = false;
  bool buzzerStatus = false;

  if (temp > 35) {
    digitalWrite(RED_LED, HIGH);
    digitalWrite(ALERT_BUZZER, HIGH);
    redLEDStatus = true;
    buzzerStatus = true;
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
  } else if (temp >= 30 && temp <= 35) {
    digitalWrite(YELLOW_LED, HIGH);
    yellowLEDStatus = true;
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(ALERT_BUZZER, LOW);
  } else {
    digitalWrite(GREEN_LED, HIGH);
    greenLEDStatus = true;
    digitalWrite(RED_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(ALERT_BUZZER, LOW);
  }

  if (temp >= 35) {
    digitalWrite(relayControlPin, HIGH);
  } else {
    digitalWrite(relayControlPin, LOW);
  }

  char tempBuffer[8];
  char humBuffer[8];
  dtostrf(temp, 1, 2, tempBuffer);
  dtostrf(hum, 1, 2, humBuffer);

  mqtt.publish("sensor/temperature", tempBuffer);
  mqtt.publish("sensor/humidity", humBuffer);

  String statusMsg = String("Red LED: ") + (redLEDStatus ? "ON" : "OFF") +
                     ", Yellow LED: " + (yellowLEDStatus ? "ON" : "OFF") +
                     ", Green LED: " + (greenLEDStatus ? "ON" : "OFF") +
                     ", Buzzer: " + (buzzerStatus ? "ON" : "OFF") +
                     ", Relay: " + ((temp >= 35) ? "ON" : "OFF");
  mqtt.publish("sensor/status", statusMsg.c_str());

  delay(2000);
}
