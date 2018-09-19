// rfm69 - mqtt gateway
// for ESP8266 (and ESP32)

#include <Arduino.h>
#include <stdint.h>

#include <SPI.h>
#include "RF69min.h"

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include <ArduinoJson.h>
#include "jsonlpp.h"
// #include <base64.hpp>


// WiFi
const char* ssid     = "...";
const char* password = "...";
unsigned char mac[6];
WiFiClient espClient;

// PubSub (MQTT)
const char* mqttServer = "myBroker";
// alternative: IPAddress mqttServer(172, 16, 0, 2);
const int mqttPort = 1883;
const char mqttUser[] ="myUsername";
const char mqttPassword[] ="myPassword";

PubSubClient client(espClient);

// gateway also used as node

String nodeID;
String actuatorTopic;
int msgCount = 0;
const int maxNode = 64;
String nodeMap[maxNode];

unsigned long sensor1TimerStart = 0;
unsigned long sensor1Period = 60000UL; // in millisecs

// RFM69 interface
// Report received data on the serial port.

RF69 rf (D0);                // CS pin
uint8_t RF69_resetpin = D8;  // Reset (Adafruit module)

uint8_t rxBuf[64];
uint8_t txBuf[62];
uint16_t cnt = 0;

bool isHighPower = true;

// Network/WiFi

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
  }
  Serial.println();
  digitalWrite(LED_BUILTIN, HIGH); // LED off
}

void mqttReconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientID = "IoTClient-" + nodeID;
    if (client.connect(clientID.c_str(), mqttUser, mqttPassword)) {
      Serial.println(" connected");

      //client.subscribe(actuatorTopic.c_str());
      client.subscribe("node/+/actuators");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// MQTT - publish

void selfPublish() {
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  String gatewayTopic = "gateway/" + nodeID;
  String msg;

  root["id"] = nodeID;
  root["msgCount"] = msgCount;
  root["localtime"] = millis();
  root.printTo(msg);
  Serial.println(msg);
  client.publish(gatewayTopic.c_str(), msg.c_str());
}

// mqtt
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  // split topic in node/ - ID - /actuators
  String topicStr(topic);
  int firstSlash = topicStr.indexOf('/');
  int nextSlash = topicStr.indexOf('/', firstSlash+1);
  String topicFront = topicStr.substring(0, firstSlash + 1);
  String topicID = topicStr.substring(firstSlash + 1, nextSlash);
  String topicTail = topicStr.substring(nextSlash, -1); // to end of string  
  
  if (topicFront == "node/" && topicTail== "/actuators") {   
    Serial.print("Actuator meesage received for node: ");
    Serial.println(topicID);

    // find local RFM address for node topicID:
    uint8_t dstNode = 0;
    while (dstNode < maxNode && nodeMap[dstNode] != topicID) {
      dstNode++;
    }
    if (nodeMap[dstNode] == topicID) {
      
      // convert JSON payload to LPP:
      // first, copy to enforce a zero-terminated string.
      char json[64];
      for (int i = 0; i < length; i++) {
        json[i] = (char) payload[i];
      }
      json[length] = '\0'; 

      // parse JSON string
      StaticJsonBuffer<256> jsonBuffer;
      JsonObject& root = jsonBuffer.parseObject(json);
      if (!root.success()) {
        Serial.println("json parse failed");
        return;
      }

      // convert to lpp:
      uint8_t lppBuffer[64];
      int len = jsonToLpp(root, lppBuffer);

      Serial.print("Resulting lpp-buffer: ");
      for (int i = 0; i<len; i++) {
        Serial.print(lppBuffer[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      
      // String msg = payload;
      Serial.println(json);
      Serial.print("Send to node: ");
      Serial.println(dstNode);
      rf.send(dstNode, lppBuffer, len);
    } else {
      Serial.println("Dest node not found...");
    }
  }
}

// RFM69

#define LPP_port 1 // LPP protocol on port 1

void rfReceive(uint8_t *msg, uint8_t len, uint8_t rssi) {
  uint8_t dst = msg[0];
  uint8_t src = msg[1];
  uint8_t port = msg[2];
  String mqttMsg;

  if (port == LPP_port) {
    unsigned int nodeid = (msg[3] << 8) | msg[4];
    String nodeidString = String(nodeid, HEX);
    nodeMap[src] = nodeidString;  // for routing of downlink-messages   
    unsigned int counter = (msg[5] << 8) | msg[6];
    // check counter incrementing !!!

    StaticJsonBuffer<512> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    
    // metadata:
    root["nodeid"] = nodeidString;
    root["counter"] = counter;
    root["rssi"] = rssi;
    // and other metadata - a.o time!
    
    // payload:
    //uint8_t buf[64];
    //for (int i = 0; i < len - 7; i++) {
    //  buf[i] = msg[i+7];
    //}
    JsonArray& payload_raw = root.createNestedArray("payload_raw");
    for (int i = 0; i < len - 7; i++) {
      payload_raw.add( msg[i+7]);
    }

//    uint8_t buf[64];
//    int bufLen = encode_base64(&msg[7], len - 7, buf);
//    root["payload_raw"] = buf;
       
    JsonObject& payload = root.createNestedObject("payload"); 
    lppToJson(&msg[7], len - 7, payload);

    // construct & send mqtt-message
    root.printTo(mqttMsg);
    String sensorTopic = "node/" + nodeidString + "/sensors";  
    Serial.print("Publish: ");
    Serial.print(sensorTopic);
    Serial.print(" ");
    Serial.println(mqttMsg);
    char msg [200];
    for (int i = 0; i < mqttMsg.length(); i++) {
      msg[i] = mqttMsg[i];
    }
    // msg[mqttMsg.length()] = '\0';
    msg[105]= '\0';
    bool res = client.publish(sensorTopic.c_str(), mqttMsg.c_str());  
    // bool res = client.publish("node/abcd/sensors", msg);
    Serial.print("Success: ");
    Serial.println(res);
    msgCount = msgCount + 1;    
  } else { 
    // ignore unknown format, print msg in hex and ascii
    Serial.println("Unknown format received:");
    for (int i = 0; i < len; i++) {
      Serial.print(msg[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    for (int i = 0; i < len; i++) {
      if (msg[i] >= (int)' ') {
        Serial.print((char)msg[i]);
      }
    }
    Serial.println();
  }
}

void rfSetup() {
  pinMode(RF69_resetpin, OUTPUT);
  digitalWrite(RF69_resetpin, LOW);
  delay(10);
  digitalWrite(RF69_resetpin, HIGH);
  delay(10);
  digitalWrite(RF69_resetpin, LOW);
  delay(100);
  SPI.begin();
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0)); // 10 MHz
  rf.init(1, 42, 8686);
  Serial.println("after RF69 init");
  //rf.encrypt("mysecret");
  rf.txPower(31, isHighPower); // 0 = min .. 31 = max  
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  Serial.println();
  Serial.println("[iot2018-gateway (LPP)]");

  Serial.print("Max mqtt pkt size: ");
  Serial.println(MQTT_MAX_PACKET_SIZE);

  rfSetup();
 
  networkSetup();
  nodeID = String(mac[4] * 256 + mac[5], HEX);
  // MQTT init:
  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttCallback); 
}

void loop() {

  if (!client.connected()) {
    mqttReconnect();
  }
  client.loop();

  if (millis() - sensor1TimerStart >= sensor1Period) {
    selfPublish();
    txBuf[0] = 255; // channel nr.
    txBuf[1] = 0;
    rf.send(0, txBuf, 2); // broadcast.
    sensor1TimerStart = sensor1TimerStart + sensor1Period;
  }

  int len = rf.receive(rxBuf, sizeof rxBuf);
  if (len >= 0) {
    Serial.print("RFM msg received, rssi: ");
    Serial.println(rf.rssi);
    rfReceive(rxBuf, len, rf.rssi);
    // rf.afc; rf.lna
  } 
}
