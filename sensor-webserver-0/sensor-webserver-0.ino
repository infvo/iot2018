
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
// for BMP280:
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

const char* ssid = "...";            // adapt to local WiFi
const char* password = "..."; // adapt to local WiFi
unsigned char mac[6];

const int ledPin = D5;

// Create an instance of the server
// specify the port to listen on as an argument
ESP8266WebServer server(80);

Adafruit_BME280 bme; // I2C

void sendHTMLdoc() {
  digitalWrite(LED_BUILTIN, LOW);
  const char *red = "red";
  const char *black = "black";
  const char *color;
  char buffer[800];

  if (digitalRead(ledPin)) {
    color = red;
  } else {
    color = black;
  }

  float temp = bme.readTemperature();
  float pres = bme.readPressure() / 100;

  snprintf ( buffer, 800,
"<html>\n\
  <head>\n\
      <title>ESP8266 Sensor server</title>\n\
      <style>\n\
        table, th, td {border: 1px;}\n\
        table {border-collapse: collapse}\n\
      </style>\n\
  </head>\n\
  <body> <h1>ESP8266 control</h1>\n\
    <p>\n\
      <form action=\"/leds/0\" method=\"post\">\n\
         <button type=\"submit\" name=\"on\" value=\"1\">On</button>\n\
         <span style=\"font-weight:bold;color:%s;\"> [[LED]] </span>\n\
         <button type=\"submit\" name=\"on\" value=\"0\">Off</button>\n\
      </form>\n\
    </p>\n\
    <table>\n\
        <tr><td>Temperature</td>   <td>%.02f &deg;C</td></tr>\n\
        <tr><td>Atm.pressure</td>  <td>%.02f hPa</td> </tr>\n\
    </table>\n\
    <p><a href=\"/\">refresh</a></p>\n\
  </body>\n\
</html>\n",
    color, temp, pres
  );
  server.send ( 200, "text/html", buffer );
  digitalWrite(LED_BUILTIN, HIGH); 
}

void handleRoot() {
  Serial.println("/");
  sendHTMLdoc();
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
          digitalWrite(ledPin, LOW);          
        } else if (argvalue == "1") {
          digitalWrite(ledPin, HIGH);
        }
      }
    }  
  }
  sendHTMLdoc();
}

void handleNotFound(){
  digitalWrite(LED_BUILTIN, LOW);
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  digitalWrite(LED_BUILTIN, HIGH);
}

void setup() {
  Serial.begin(115200);
  delay(10);

  // prepare led
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Wire.begin(4, 5); // SDA, SCL: GPIO nrs - D2, D1 - I2C for BMP
  while (!bme.begin(0x76)) {
    digitalWrite(LED_BUILTIN, LOW); // active low: LED ON 
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH); // active low: LED OFF 
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    delay(10000);
  }
  
  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  for (int i = 0; i < 6; i++) {
    int macbyte = mac[i];
    if (macbyte < 16) {
      Serial.print('0');
    }
    Serial.print(macbyte, HEX);
  }
  Serial.println();
  String domainname = "esp8266-" + String(mac[4] * 256 + mac[5], HEX);
  if (MDNS.begin(domainname.c_str())) {
    Serial.print("mDNS responder started for: ");
    Serial.println(domainname);
  }

  server.on("/", handleRoot);
  server.on("/leds/0", handleLed0);
  server.onNotFound(handleNotFound);
    
  // Start the server
  server.begin();
  Serial.println("HTTP server started");
  
  digitalWrite(LED_BUILTIN, HIGH); // setup complete
}

void loop() {
  server.handleClient();  
}

