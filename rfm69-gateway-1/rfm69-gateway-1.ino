// rfm69 - mqtt gateway
// for ESP8266 (and ESP32)

#include <Arduino.h>
#include <stdint.h>

#include <SPI.h>
#include "RF69min.h"

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <FS.h>
// MQTT
#include <PubSubClient.h>

#include <ArduinoJson.h>
#include "jsonlpp.h"
// #include <base64.hpp>

// Configuration
String wifiSsid = "";
String wifiPassword = "";
boolean mqttActive = false;
const int mqttPort = 1883;
String mqttServer = "";
String mqttUser = "";
String mqttPassword = "";

// i/o pin map
const int led0 = D3;
const int led1 = D5;
const int button0 = D6; 
const int button1 = D7;

// WiFi
unsigned char mac[6];
String nodeID;
IPAddress myIP;
ESP8266WebServer server(80);
WiFiClient espClient;

// PubSub (MQTT)
PubSubClient client(espClient);

// configuration

bool loadConfig() {
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println("Failed to open config file");
    return false;
  }

  size_t size = configFile.size();
  if (size >= 1024) {
    Serial.println("Config file size is too large");
    return false;
  }

  // Allocate a buffer to store contents of the file.
  char buf [1024];
  configFile.readBytes(buf, size);
  buf[size] = 0;

  Serial.println("Load config file:");
  Serial.println(buf);  

  StaticJsonBuffer<400> jsonBuffer;
  JsonObject& json = jsonBuffer.parseObject(buf);

  if (!json.success()) {
    Serial.println("Failed to parse config file");
    return false;
  }

  wifiSsid = json.get<String>("wifi_ssid");
  wifiPassword = json.get<String>("wifi_password");
  mqttServer = json.get<String>("mqtt_broker");
  mqttUser = json.get<String>("mqtt_user");
  mqttPassword = json.get<String>("mqtt_password");
  return true;
}

bool saveConfig() {
  StaticJsonBuffer<400> jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["wifi_ssid"] = wifiSsid;
  json["wifi_password"] = wifiPassword;
  json["mqtt_broker"] = mqttServer;
  json["mqtt_user"] = mqttUser;
  json["mqtt_password"] = mqttPassword;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
    return false;
  }

  json.printTo(configFile);
  configFile.close();
  Serial.println("Saved configfile:");
  json.printTo(Serial);
  Serial.println();
  return true;
}


// webserver

String ipAddress2String(const IPAddress& ipAddress) {
  return String(ipAddress[0]) + String(".") +
    String(ipAddress[1]) + String(".") +
    String(ipAddress[2]) + String(".") +
    String(ipAddress[3]); 
}

void handleSetup() {
    int hr = 2, min = 3, sec = 4;
    char temp[1000];
    Serial.println("setup");
    snprintf(temp, 1000,

           "<!DOCTYPE html\n\
  <html>\n\
  <head>\n\
    <title>ESP8266 setup</title>\n\
    <meta content=\"width=device-width, initial-scale=1\" name=\"viewport\"/>\n\
    <style>\n\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\n\
    </style>\n\
  </head>\n\
  <body>\n\
    <h1>ESP8266 IoT-node (%s)</h1>\n\
    <form action=\"/setupform\" method=\"post\">\n\
       wifi_ssid: <input type=\"text\" name=\"ssid\" value=\"%s\"><br>\n\
       wifi_password: <input type=\"text\" name=\"password\" value=\"\"><br>\n\
       mqtt_broker: <input type=\"text\" name=\"mqtt_broker\" value=\"%s\"><br>\n\
       mqtt_user: <input type=\"text\" name=\"mqtt_user\" value=\"%s\"><br>\n\
       mqtt_password: <input type=\"text\" name=\"mqtt_password\" value=\"\"><br>\n\
       <input type=\"submit\" value=\"Submit\">\n\
    </form>\n\
    <p> <a href=\"/\">Home</a> --- <a href=\"/reset\">[[Reset]]</a> </p>\n\
  </body>\n\
</html>",
  nodeID.c_str(),
  wifiSsid.c_str(),
  mqttServer.c_str(),
  mqttUser.c_str(), mqttPassword.c_str()
          );
  server.send(200, "text/html", temp);
}

void handleReset() {
  Serial.println("reset");
  String html = "trying to connect upon reset...";
  server.send(200, "text/html", html.c_str());
  delay(1000);  
  ESP.restart();
}

void handleSetupForm() {
  Serial.println("/setupform");
  if (server.method() == HTTP_POST) {
    for (uint8_t i=0; i < server.args(); i++) {
      if (server.argName(i) == "ssid") {
        wifiSsid = server.arg(i);
      } else if (server.argName(i) == "password") {
        if (server.arg(i).length() > 0 || wifiSsid.length() == 0) {
          wifiPassword = server.arg(i); 
        }  // else keep current password 
      } else if (server.argName(i) == "mqtt_broker") {
        mqttServer = server.arg(i);    
      } else if (server.argName(i) == "mqtt_user") {
        mqttUser = server.arg(i);     
      } else if (server.argName(i) == "mqtt_password") {
        if (server.arg(i).length() > 0 || mqttUser.length() == 0) {
          mqttPassword = server.arg(i); 
        } // else keep current password           
      }     
    }  
  }
  saveConfig();
  handleSetup();
}

void handleNotFound() {
//  digitalWrite(led, 1);
  Serial.print("Not found: ");
  Serial.println(server.uri());
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }

  server.send(404, "text/plain", message);
//  digitalWrite(led, 0);
}

// access point etc.

// setup as WiFi access point
void setupAP() {
  Serial.println("Configuring wifi access point...");
  /* Leaving out the password parameter for open AP */
  WiFi.mode(WIFI_AP);

  String apSsid = "ESPAP-" + nodeID;
  WiFi.softAP(apSsid.c_str());
  Serial.println("Access point-ssid:" + apSsid);
  myIP = WiFi.softAPIP();
}

// setup as WiFI station
void setupSTA() {
  digitalWrite(LED_BUILTIN, LOW); // on
  Serial.print("Connecting to ");
  Serial.println(wifiSsid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  myIP = WiFi.localIP();
  digitalWrite(LED_BUILTIN, HIGH); // off
}

void setupServer() {
  server.on("/setupform", handleSetupForm);
  server.on("/reset", handleReset);
  server.on("/", handleSetup);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");  
}


// gateway also used as node

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

// MQTT

void mqttReconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientID = "IoTClient-" + nodeID;
    if (client.connect(clientID.c_str(), mqttUser.c_str(), mqttPassword.c_str())) {
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

void setupMQTT() {
  Serial.println("MQTT connection");
  // sensorTopic = "node/" + nodeID + "/sensors";
  actuatorTopic = "node/" + nodeID + "/actuators";
  client.setServer(mqttServer.c_str(), mqttPort);
  client.setCallback(mqttCallback);  
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
  pinMode(led0, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(button0, INPUT);
  if (digitalRead(button0) == HIGH) {
    digitalWrite(led0, HIGH);
  }

  Serial.begin(115200);
  Serial.println();
  Serial.println("[iot2018-gateway v0.1 (LPP)]");

  Serial.print("Max mqtt pkt size: ");
  Serial.println(MQTT_MAX_PACKET_SIZE);

// get configuration
  boolean config_OK = false;
  if (SPIFFS.begin()) {
   config_OK = loadConfig();
  } else {
    Serial.println("Failed to mount file system");
  }
  mqttActive = mqttServer.length() > 0;

// nodeID
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(mac[i], HEX);
    Serial.print(":");
  }
  Serial.println();
  nodeID = String(mac[4] * 256 + mac[5], HEX);

// start as wifi access point or as station
  if (digitalRead(button0) || !config_OK) {
    digitalWrite(led1, HIGH);
    setupAP();
    mqttActive = false;  
  } else {
    setupSTA();
  }
  Serial.print("My IP address: ");
  Serial.println(myIP);



  setupServer();
  
  if (mqttActive) {
    setupMQTT();
    rfSetup();
  } else {
    Serial.println("no MQTT broker");
  }

}

void loop() {

  if (mqttActive) {
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
  } else {
    server.handleClient();
  }
}
