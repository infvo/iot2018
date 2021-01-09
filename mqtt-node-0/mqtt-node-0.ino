/*
 * test a combination of JSON, MQTT over ESP8266-WiFi
 * simple sensor: button
 * simple actuator: LED
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
// for BMP280:
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// i/o pin map
const int led0 = D3;
const int led1 = D5;
const int button0 = D6; 
const int button1 = D7;

Adafruit_BME280 bme; // I2C

// WiFi
const char* ssid     = "...";
const char* password = "...";
unsigned char mac[6];
WiFiClient espClient;

// PubSub (MQTT)
const char* mqttServer = "...";
// alternative: IPAddress mqttServer(172, 16, 0, 2);
const int mqttPort = 1883;
const char mqttUser[] ="...";
const char mqttPassword[] ="...";

PubSubClient client(espClient);

String nodeID;
String sensorTopic;
String actuatorTopic;

unsigned int counter = 0;

long sensor1TimerStart = 0;
long sensor1TimerPeriod = 30000L; // in millisecs

long button0TimerStart = 0;
long button0TimerPeriod = 1000L; // in millisecs
long button1TimerStart = 0;
long button1TimerPeriod = 1000L; // in millisecs  

// JSON

void sensor0Publish() {
  StaticJsonDocument<400> doc;
  String msg;
  doc["nodeid"] = nodeID;
  doc["counter"] = counter;
  counter = counter + 1;

  JsonObject payload = doc.createNestedObject("payload");
  
  JsonObject channel2 = payload.createNestedObject("2");
  channel2["dIn"] = digitalRead(button0);   
  JsonObject channel3 = payload.createNestedObject("3");
  channel3["dIn"] = digitalRead(button1);   

  serializeJson(doc, msg); // appends!
  Serial.println(msg);
  client.publish(sensorTopic.c_str(), msg.c_str());
}

void sensor1Publish() {
  StaticJsonDocument<400> doc;
  String msg;
  doc["nodeid"] = nodeID;
  doc["counter"] = counter;
  counter = counter + 1;

  JsonObject payload = doc.createNestedObject("payload");
  
  JsonObject channel0 = payload.createNestedObject("0");
  channel0["dOut"] = digitalRead(led0);
  JsonObject channel1 = payload.createNestedObject("1");
  channel1["dOut"] = digitalRead(led1);
  
  JsonObject channel4 = payload.createNestedObject("4");
  channel4["temperature"] = (int) (bme.readTemperature() * 10);
  JsonObject channel5 = payload.createNestedObject("5");
  channel5["barometer"] = (int) (bme.readPressure() / 10.0);
  JsonObject channel6 = payload.createNestedObject("6");
  channel6["humidity"] = (int) (bme.readHumidity() * 2);

  JsonObject channel8 = payload.createNestedObject("8");
  channel8["aIn"] = analogRead(A0);
  
  serializeJson(doc, msg); // appends!
  Serial.println(msg);
  client.publish(sensorTopic.c_str(), msg.c_str());
}

void networkSetup() {
  digitalWrite(LED_BUILTIN, LOW); // active low: LED ON
  delay(100);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.print(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 
  Serial.println();
  Serial.print("WiFi connected, IP address: ");  
  Serial.println(WiFi.localIP());
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(mac[i], HEX);
    Serial.print(":");
  }
  Serial.println();

  digitalWrite(LED_BUILTIN, HIGH); // LED off
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (strcmp(topic, actuatorTopic.c_str())==0) {
    Serial.println("actuator message received");
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, (char*) payload);

    if (!error) {
      if (doc.containsKey("0")) {
        digitalWrite(led0, doc["0"]["dOut"]);
      }
      if (doc.containsKey("1")) {
        digitalWrite(led1, doc["1"]["dOut"]);
      }
      sensor1Publish();
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientID = "IoTClient-" + nodeID;
    if (client.connect(clientID.c_str(), mqttUser, mqttPassword)) {
      Serial.println("connected");
// Serial.print("Subscribe: "); Serial.println(actuatorTopic);
      client.subscribe(actuatorTopic.c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  pinMode(led0, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(button0, INPUT);
  Wire.begin(D2, D1); // SDA, SCL: GPIO nrs - D2(GPIO4), D1(GPIO5) - I2C for BMP
  Serial.begin(115200);
  Serial.println();
  Serial.println("[iot2018-mqtt-node (bme)]");
  while (!bme.begin(0x76)) {
    digitalWrite(LED_BUILTIN, LOW); // active low: LED ON 
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH); // active low: LED OFF 
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    delay(10000);
  }
  
  networkSetup();
  nodeID = String(mac[4] * 256 + mac[5], HEX);
  // MQTT init:
  sensorTopic = "node/" + nodeID + "/sensors";
  actuatorTopic = "node/" + nodeID + "/actuators";
  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttCallback);
}

void loop() {
  if (!client.connected()) {
    Serial.print("...not connected... ");
    reconnect();
  }
  client.loop();

  if ((millis() - button0TimerStart >= button0TimerPeriod) && 
      (digitalRead(button0) == 1)) {
    delay(2);
    if (digitalRead(button0) == 1) {
      sensor0Publish();
      button0TimerStart = millis();    
    }     
  }

  if ((millis() - button1TimerStart >= button1TimerPeriod) && 
      (digitalRead(button1) == 1)) {
    delay(2);
    if (digitalRead(button1) == 1) {
      sensor0Publish();
      button1TimerStart = millis();    
    }     
  }

  if (millis() -  sensor1TimerStart >= sensor1TimerPeriod) {
    sensor1Publish();
    sensor1TimerStart = sensor1TimerStart + sensor1TimerPeriod;
    if ((counter == 10) || (counter == 100) || (counter == 1000)) {
      Serial.print("Double timer period:" );
      Serial.println(sensor1TimerPeriod * 2L);
      sensor1TimerPeriod = sensor1TimerPeriod * 2L;
    }    
  }
}
