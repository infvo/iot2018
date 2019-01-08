
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
// MQTT
#include <PubSubClient.h>
// JSON, config
#include <ArduinoJson.h>
#include "FS.h"
// for BMP280:
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme; // I2C

unsigned char mac[6];
String nodeID;
IPAddress myIP;
ESP8266WebServer server(80);
WiFiClient espClient;
PubSubClient client(espClient);

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

// configuration:

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

void handleLedOn() {
  Serial.println("ledOn");
  digitalWrite(led0, HIGH);
  sendHTMLdoc();
}

void handleLedOff() {
  Serial.println("ledOff");
  digitalWrite(led0, LOW);
  sendHTMLdoc();  
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
  sendHTMLdoc();
}

void sendHTMLdoc() {
  digitalWrite (LED_BUILTIN, LOW );
  const char *red = "red";
  const char *black = "black";
  const char *color;
  char buffer[1200];

  if (digitalRead(led0)) {
    color = red;
  } else {
    color = black;
  }

  float temp = bme.readTemperature();
  float pres = bme.readPressure() / 100;
  int hum = bme.readHumidity();
  int light = analogRead(A0);
  
  snprintf ( buffer, 1200,
"<html>\n\
  <head>\n\
    <title>ESP8266 sensors</title>\n\
    <meta content=\"width=device-width, initial-scale=1\" name=\"viewport\"/>\n\
    <style>\n\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\n\
    </style>\n\
  </head>\n\
  <body> <h1>ESP8266 (%s) sensors</h1>\n\
    <p>\n\
      <form action=\"/leds/0\" method=\"post\">\n\
         <button type=\"submit\" name=\"on\" value=\"1\">On</button>\n\
         <span style=\"font-weight:bold;color:%s;\"> [[LED]] </span>\n\
         <button type=\"submit\" name=\"on\" value=\"0\">Off</button>\n\
      </form>\n\
    </p>\n\
    <table>\n\
        <tr><td>Temperature</td> <td>%.02f &deg;C</td></tr>\n\
        <tr><td>Barometer</td> <td>%.02f hPa</td> </tr>\n\
        <tr><td>Humidity</td> <td>%2d %%</td> </tr>\n\
        <tr><td>Light</td> <td>%4d </td> </tr>\n\
        <tr><td>IP address</td> <td>%s</td> </tr>\n\
    </table>\n\
    <p> <a href=\"/\">Home</a> --- <a href=\"/setup\">Setup</a> </p>\n\
  </body>\n\
</html>\n",
    nodeID.c_str(),
    color, temp, pres, hum, light,
    ipAddress2String(WiFi.localIP()).c_str()
  );
  server.send ( 200, "text/html", buffer );
  digitalWrite ( LED_BUILTIN, HIGH ); 
}

void handleLed0() {
  Serial.print("/leds/0 ");
  if (server.method() == HTTP_POST) {
    for (uint8_t i=0; i < server.args(); i++) {
      if (server.argName(i) == "on") {
        String argvalue = server.arg(i);
        Serial.print("on=");
        Serial.println(argvalue);
        if (argvalue == "0") {
          digitalWrite(led0, LOW);
          Serial.println("led 0 low");        
        } else if (argvalue == "1") {
          digitalWrite(led0, HIGH);
          Serial.println("led 0 high");
        }
      }
    }  
  }
  sendHTMLdoc();
}

void handleRoot() {
  Serial.println("root");
  sendHTMLdoc();
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

// PubSub (MQTT)

String sensorTopic;
String actuatorTopic;

unsigned int counter = 0;

long sensor1TimerStart = 0;
long sensor1TimerPeriod = 30000L; // in millisecs

long button0TimerStart = 0;
long button0TimerPeriod = 1000L; // in millisecs
long button1TimerStart = 0;
long button1TimerPeriod = 1000L; // in millisecs

void sensor0Publish() {
  StaticJsonBuffer<400> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  String msg;
  root["nodeid"] = nodeID;
  root["counter"] = counter;
  counter = counter + 1;

  JsonObject& payload = root.createNestedObject("payload");
  
  JsonObject& channel2 = payload.createNestedObject("2");
  channel2["dIn"] = digitalRead(button0);   
  JsonObject& channel3 = payload.createNestedObject("3");
  channel3["dIn"] = digitalRead(button1);   

  root.printTo(msg);
  Serial.println(msg);
  client.publish(sensorTopic.c_str(), msg.c_str());
}

void sensor1Publish() {
  StaticJsonBuffer<400> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  String msg;
  root["nodeid"] = nodeID;
  root["counter"] = counter;
  counter = counter + 1;

  JsonObject& payload = root.createNestedObject("payload");
  
  JsonObject& channel0 = payload.createNestedObject("0");
  channel0["dOut"] = digitalRead(led0);
  JsonObject& channel1 = payload.createNestedObject("1");
  channel1["dOut"] = digitalRead(led1);
  
  JsonObject& channel4 = payload.createNestedObject("4");
  channel4["temperature"] = (int) (bme.readTemperature() * 10);
  JsonObject& channel5 = payload.createNestedObject("5");
  channel5["barometer"] = (int) (bme.readPressure() / 10.0);
  JsonObject& channel6 = payload.createNestedObject("6");
  channel6["humidity"] = (int) (bme.readHumidity() * 2);

  JsonObject& channel8 = payload.createNestedObject("8");
  channel8["aIn"] = analogRead(A0);
  
  root.printTo(msg);
  Serial.println(msg);
  client.publish(sensorTopic.c_str(), msg.c_str());
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
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject((char*) payload);
    if (root.success()) {
      if (root.containsKey("0")) {
        digitalWrite(led0, root["0"]["dOut"]);
      }
      if (root.containsKey("1")) {
        digitalWrite(led1, root["1"]["dOut"]);
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
    if (client.connect(clientID.c_str(), mqttUser.c_str(), mqttPassword.c_str())) {
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
  Serial.print("Connecting to wifi network:");
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
  
  String domainname = "esp8266-" + nodeID;
  if (MDNS.begin(domainname.c_str())) {
  Serial.print("mDNS responder started for: ");
    Serial.println(domainname);
  }
}

void setupServer() {
  server.on("/setup", handleSetup);
  server.on("/setupform", handleSetupForm);
  server.on("/reset", handleReset);
  server.on("/leds/0", handleLed0);
  server.on("/ledon", handleLedOn);
  server.on("/ledoff", handleLedOff);
  server.on("/", handleRoot);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");  
}

void setupMQTT() {
  Serial.println("MQTT connection");
  sensorTopic = "node/" + nodeID + "/sensors";
  actuatorTopic = "node/" + nodeID + "/actuators";
  client.setServer(mqttServer.c_str(), mqttPort);
  client.setCallback(mqttCallback);  
}

void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial.println();
  Serial.println("[esp8266 iot-node v2081126c]");
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  pinMode(led0, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(button0, INPUT);
  if (digitalRead(button0) == HIGH) {
    digitalWrite(led0, HIGH);
  }
  Wire.begin(D2, D1); // SDA, SCL: GPIO nrs - D2(GPIO4), D1(GPIO5) - I2C for BMP

  while (!bme.begin(0x76)) {
    digitalWrite(LED_BUILTIN, LOW); // active low: LED ON 
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH); // active low: LED OFF 
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    delay(10000);
  }  
  
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
  } else {
    Serial.println("no MQTT broker");
  }
}

void loop() {
  server.handleClient();

  if (mqttActive) {
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
}
