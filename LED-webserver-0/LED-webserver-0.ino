
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

const char* ssid = "...";            // adapt to local WiFi
const char* password = "..."; // adapt to local WiFi
unsigned char mac[6];

const int ledPin = D5;
bool ledOn = false;

// Create an instance of the server
// specify the port to listen on as an argument
ESP8266WebServer server(80);

void sendHTMLdoc() {
  digitalWrite (LED_BUILTIN, LOW );
  const char *red = "red";
  const char *black = "black";
  const char *color;
  char temp[400];

  if (ledOn) {
    color = red;
  } else {
    color = black;
  }

  snprintf ( temp, 400,
"<html>\n\
  <head> <title>ESP8266 Led server</title> </head>\n\
  <body> <h1>ESP8266 Led control</h1>\n\
    <p>\n\
      <a href=\"/ledon\"> On </a> &gt;\n\
      <span style=\"font-weight:bold;color:%s;\"> [[LED]] </span> &lt;\n\
      <a href=\"/ledoff\"> Off </a>\n\
    </p>\n\
  </body>\n\
</html>\n",
    color
  );
  server.send ( 200, "text/html", temp );
  digitalWrite ( LED_BUILTIN, HIGH );
}

void handleRoot() {
  sendHTMLdoc();
}

void handleLedOn() {
  digitalWrite(ledPin, HIGH);
  ledOn = true;
  sendHTMLdoc();
}

void handleLedOff() {
  digitalWrite(ledPin, LOW);
  ledOn = false;
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
    Serial.print(mac[i], HEX);
  }
  Serial.println();
  String domainname = "esp8266-" + String(mac[4] * 256 + mac[5], HEX);
  if (MDNS.begin(domainname.c_str())) {
    Serial.print("mDNS responder started for: ");
    Serial.println(domainname);
  }

  server.on("/", handleRoot);
  server.on("/ledon", handleLedOn);
  server.on("/ledoff", handleLedOff);
  server.onNotFound(handleNotFound);

  // Start the server
  server.begin();
  Serial.println("HTTP server started");

  digitalWrite(LED_BUILTIN, HIGH); // setup complete
}

void loop() {
  server.handleClient();
}
