
 /*
 * RFM69 iot-node
 * for Adafruit m0 combined with swing board
 * simple sensor: button
 * simple actuator: LED
 * temperature, barometer: BMP280
 */

// for configuration:
#include <FlashAsEEPROM.h>
// for RFM69:
#include <SPI.h>
#include "RF69min.h"
// for BMP280:
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// LPP format:
#define LPP_DIGITAL_INPUT       0       // 1 byte
#define LPP_DIGITAL_OUTPUT      1       // 1 byte
#define LPP_ANALOG_INPUT        2       // 2 bytes, 0.01 signed
#define LPP_ANALOG_OUTPUT       3       // 2 bytes, 0.01 signed
#define LPP_LUMINOSITY          101     // 2 bytes, 1 lux unsigned
#define LPP_PRESENCE            102     // 1 byte, 1
#define LPP_TEMPERATURE         103     // 2 bytes, 0.1°C signed
#define LPP_RELATIVE_HUMIDITY   104     // 1 byte, 0.5% unsigned
#define LPP_ACCELEROMETER       113     // 2 bytes per axis, 0.001G
#define LPP_BAROMETRIC_PRESSURE 115     // 2 bytes 0.1 hPa Unsigned
#define LPP_GYROMETER           134     // 2 bytes per axis, 0.01 °/s
#define LPP_GPS                 136     // 3 byte lon/lat 0.0001 °, 3 bytes alt 0.01m

// i/o pin map
const int led0 = A5; // or pin 19
const int led1 = A3; // on sensor connector
const int button0 = 6; // active-low, use pul-lup mode
const int button1 = 9; // active-low, use pull-up mode
const int ldr = A2;

uint8_t led0State = 0;
uint8_t led1State = 1;

// Configuration
const int confMagic = 0xaa;
const int confVersion = 3;

byte nodeAddr = 4;       // address in (local) RFM69 network
unsigned int nodeID = 0xff04; // nodeID in MQTT topic

Adafruit_BME280 bme; // I2C

// RFM69
RF69 rf (8);               // M0-RFM69 combo
uint8_t RF69_resetpin = 4;  // M0-RFM69 combo
bool isHighPower = true;

uint8_t rxBuf[64];
uint8_t txBuf[62];
int txIndex = 0;
uint16_t cnt = 0;

// RFM69 network (see also configuration)

const int gatewaynr = 1;
const int groupnr = 42;

String sensorTopic;
String actuatorTopic;

unsigned long sensor1TimerStart = 0;
unsigned long sensor1Period = 30000UL; // in millisecs

unsigned long button0TimerStart = 0;
unsigned long button0TimerPeriod = 1000L; // in millisecs
unsigned long button1TimerStart = 0;
unsigned long button1TimerPeriod = 1000L; // in millisecs  


unsigned int counter = 0;

// Configuration

byte hexDigit (char c) {
  if ('0' <= c && c <= '9') {
    return ((byte) c) - ((byte) '0'); 
  } else if ('a' <= c && c <= 'f') { 
    return 10 + ((byte) c) - ((byte) 'a');
  } else if ('A' <= c && c <= 'F') {
    return 10 + ((byte) c) - ((byte) 'A');
  } else {
    return 255;
  }
}

bool isHexString(String s, int len) {
  if (s.length() == len) {
    for (int i = 0; i < len; i++) {
      if (hexDigit(s[i]) >= 16) {
        return false;
      }
    }
    return true;
  }
  return false;
}

bool readConfiguration() {
  if (EEPROM.isValid() && EEPROM.read(0) == confMagic) {
    if (EEPROM.read(1) == confVersion) {
      nodeAddr = EEPROM.read(2);
      nodeID = (EEPROM.read(3) << 8) | EEPROM.read(4);
      return true;
    }
  }
  return false;
}

void writeConfiguration() {
  EEPROM.write(0, confMagic);
  EEPROM.write(1, confVersion);
  EEPROM.write(2, nodeAddr);
  EEPROM.write(3, (nodeID >> 8) & 0xff);
  EEPROM.write(4, nodeID & 0xff);
  EEPROM.commit();  
}

unsigned int askHexNumber(int len) {
  String hexString;
  Serial.print("enter ");
  Serial.print(len);
  Serial.println(" hex digits");
  
  while (!Serial.available());
  hexString = Serial.readStringUntil('\n');
  hexString.trim();

  if (!isHexString(hexString, len)) {
    return 0;
  }

  Serial.println(hexString);
  unsigned int nr = 0;
  for (int i = 0; i < len; i++) {
    nr = (nr << 4) | hexDigit(hexString[i]);
  }
  return nr;
}

void askNodeAddr() {
  nodeAddr = 0;
  while (nodeAddr < 2 || nodeAddr > 60) {
    Serial.print("network address: ");
    nodeAddr = askHexNumber(2);
  }
}

void askNodeID() {
  nodeID = 0;
  while (nodeID == 0) {
    Serial.print("nodeID: ");
    nodeID = askHexNumber(4);
  }
}

void getConfig() {
  bool configMode = false;
  if (!readConfiguration()) {
    configMode = true;
  }

  if (!digitalRead(button0)) { // active low
    delay(1000);
    if (!digitalRead(button0)) {
      configMode = true;
    }
  } 
  
  if (configMode) {
    setLed0(1);
    askNodeID();
    askNodeAddr();
    writeConfiguration();
    setLed0(0);
  } 
}


// Leds

void setLed0 (uint8_t val) {
  led0State = val;
  digitalWrite(led0, led0State);
}

void setLed1 (uint8_t val) {
  led1State = val;
  digitalWrite(led1, led1State);
}

// RFM69 transmission

void putByte (uint8_t b) {
  txBuf[txIndex++] = b;
}

void putInt (int w) {
  txBuf[txIndex++] = highByte(w);
  txBuf[txIndex++] = lowByte(w);
}

void printTxBuf(int len) {
  Serial.print("Send: (");
  Serial.print(len);
  Serial.print(" bytes): ");
  for (int i = 0; i < len; i++) {
    Serial.print(txBuf[i], 16);
    Serial.print(" ");
  }
  Serial.println();
}

void sensor0Publish() {
  txIndex = 0;

  putByte(1); // portnr, for this payload format.

  char *ptr;
  putInt(nodeID);
  putInt(counter);
  counter = counter + 1;

  putByte(2); // channel 2: button0
  putByte(LPP_DIGITAL_INPUT);
  putByte(!digitalRead(button0));  // active low

  putByte(3); // channel 3: button3
  putByte(LPP_DIGITAL_INPUT);
  putByte(!digitalRead(button1));  // active low

  printTxBuf(txIndex);
  rf.send(gatewaynr, txBuf, txIndex); 
}

void sensor1Publish() {
  txIndex = 0;

  putByte(1); // portnr, for this payload format.
  
  char *ptr;
  putInt(nodeID);  // first convert to int...
  putInt(counter);
  counter = counter + 1;

  putByte(0); // channel 0: led0
  putByte(LPP_DIGITAL_OUTPUT);
  putByte(led0State);
  
  putByte(4); // channel 4: bmp temperature
  putByte(LPP_TEMPERATURE);
  int temp = bme.readTemperature() * 10;
  putInt(temp);

  putByte(5);
  putByte(LPP_BAROMETRIC_PRESSURE);
  int baro = bme.readPressure() / 10.0;
  putInt(baro);

  putByte(6);
  putByte(LPP_RELATIVE_HUMIDITY);
  int hum = bme.readHumidity() * 2;
  putByte(hum);

  putByte(8);
  putByte(LPP_ANALOG_INPUT);
  int light = analogRead(ldr);
  putInt(light);

  printTxBuf(txIndex);
  rf.send(gatewaynr, txBuf, txIndex);
}

void printRxBuf(int len) {
  Serial.print(" Received: (");
  Serial.print(len);
  Serial.print(" bytes): ");
  for (int i = 0; i < len; i++) {
    Serial.print(rxBuf[i], 16);
    Serial.print(" ");
  }
  Serial.println();
}
void rfReceive(uint8_t *msg, uint8_t len, uint8_t rssi) {

  uint8_t dst = msg[0];
  uint8_t src = msg[1];
  uint8_t port = msg[2];

  printRxBuf(len);
  if (port == 1) {
    int index = 3;
    while (index < len && msg[index] != 0xff) {
      unsigned int chan = msg[index++];
      if (chan == 0) {
        setLed0(msg[index++]);
      } else if (chan == 1) {
        setLed1(msg[index++]);  
      } else {
        Serial.print("invalid channel: ");
        Serial.println(chan);
      }
    }
  } else if (port == 255) {
    // broadcast from gateway (rest skipped)
    Serial.println("Broadcast from gateway");
    return;
  } else {
    Serial.print("invalid port: ");
    Serial.println(port);
  }
  sensor1Publish();
}

void rfSetup() {
  pinMode(RF69_resetpin, OUTPUT);
  digitalWrite(RF69_resetpin, LOW);
  delay(10);
  digitalWrite(RF69_resetpin, HIGH);
  delay(10);
  digitalWrite(RF69_resetpin, LOW);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // 1 MHz

  rf.init(nodeAddr, groupnr, 8686);
  Serial.println("after RF69 init");
  //rf.encrypt("mysecret");
  rf.txPower(31, isHighPower); // 0 = min .. 31 = max

  for (int i = 0; i < (int) sizeof txBuf; ++i) {
      txBuf[i] = i;
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  pinMode(led0, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(button0, INPUT_PULLUP);
  pinMode(button1, INPUT_PULLUP);
 // Wire.begin(4, 5); // SDA, SCL: GPIO nrs - D2, D1 - I2C for BMP
 //  UNO: i2c fixed, on pins SCL: A5, SDA: A4
  delay(2000);
  Serial.begin(115200);
  Serial.println();
  Serial.println("[m0-iot2019-mini-node-bmp-rfm69]");
  digitalWrite(LED_BUILTIN, LOW);

  getConfig();
  Serial.print("NodeID: ");
  Serial.print(nodeID, HEX);
  Serial.print(" nodeAddr: ");
  Serial.print(nodeAddr);
  Serial.print(" (0x");
  Serial.print(nodeAddr, HEX);
  Serial.println(")");
   
  while (!bme.begin(0x76)) {
    digitalWrite(LED_BUILTIN, HIGH); // LED ON
    delay(100);
    digitalWrite(LED_BUILTIN, LOW); // LED OFF
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    delay(10000);
  }
  Serial.println("bme280 found");
  rfSetup();
  
}

void loop() {

  if ((millis() - button0TimerStart >= button0TimerPeriod) && 
      (digitalRead(button0) == 0)) { // active low
    delay(2);
    if (digitalRead(button0) == 0) {
      sensor0Publish();
      button0TimerStart = millis();    
    }     
  }

  if ((millis() - button1TimerStart >= button1TimerPeriod) && 
      (digitalRead(button1) == 0)) {  // active low
    delay(2);
    if (digitalRead(button1) == 0) {
      sensor0Publish();
      button1TimerStart = millis();    
    }     
  }

  if (millis() - sensor1TimerStart >= sensor1Period) {
    sensor1Publish();
    sensor1TimerStart = sensor1TimerStart + sensor1Period;
    if (counter == 10 || counter == 100 || counter == 1000) {
      sensor1Period = sensor1Period * 2L;
    }
  }

  int len = rf.receive(rxBuf, sizeof rxBuf);
  if (len >= 0) {
    Serial.print("RFM msg received, rssi: ");
    Serial.print(rf.rssi);
    rfReceive(rxBuf, len, rf.rssi);
    // rf.afc; rf.lna
  }
  delay(20); // artificial delay, to reduce rf69 polling
}
