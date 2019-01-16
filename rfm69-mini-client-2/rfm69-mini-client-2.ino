
 /*
 * RFM69 iot-node
 * for Arduino UNO etc.
 * simple sensor: button
 * simple actuator: LED
 * temperature, barometer: BMP280
 */

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
const int led0 = 2;
// const int led1 = 7;
//const int button0 = 4;
// const int button1 = D3;

uint8_t led0State = 0;

void setLed0(uint8_t val) {
  led0State = val;
  digitalWrite(led0, led0State);
}

Adafruit_BME280 bme; // I2C

// RFM69
RF69 rf (10);               // Arduino: CS for RFM69 module
uint8_t RF69_resetpin = 1;  // Adafruit module
bool isHighPower = true;

uint8_t rxBuf[64];
uint8_t txBuf[62];
int txIndex = 0;
uint16_t cnt = 0;

// node id etc.

String nodeID = "ff02";  // appl.node-ID, must be 4-digit hex number
const int nodenr = 2;    // RFM network-id
const int gatewaynr = 1;
const int groupnr = 42;

String sensorTopic;
String actuatorTopic;

unsigned long sensor1TimerStart = 0;
unsigned long sensor1Period = 30000UL; // in millisecs
unsigned int counter = 0; 

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

void sensor1Publish() {
  txIndex = 0;

  putByte(1); // portnr, for this payload format.
  
  char *ptr;
  unsigned int nodeIDint = strtol(nodeID.c_str(), &ptr, 16);
  putInt(nodeIDint);  // first convert to int...
  putInt(counter);
  counter = counter + 1;

  putByte(0); // channel 0: led0
  putByte(LPP_DIGITAL_OUTPUT);
  putByte(led0State);
  
  putByte(4); // channel 3: bme temperature
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
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(RF69_resetpin, OUTPUT);
  digitalWrite(RF69_resetpin, LOW);
  delay(10);
  digitalWrite(RF69_resetpin, HIGH);
  delay(10);
  digitalWrite(RF69_resetpin, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // 1 MHz

  rf.init(nodenr, groupnr, 8686);
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
// pinMode(led1, OUTPUT);
// pinMode(button0, INPUT);
 // Wire.begin(4, 5); // SDA, SCL: GPIO nrs - D2, D1 - I2C for BMP
 //  UNO: i2c fixed, on pins SCL: A5, SDA: A4
  Serial.begin(115200);
  Serial.println();
  Serial.println("[iot2018-mini-node-bme-rfm69]");
  while (!bme.begin(0x76)) {
    digitalWrite(LED_BUILTIN, HIGH); // active low: LED ON
    delay(100);
    digitalWrite(LED_BUILTIN, LOW); // active low: LED OFF
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    delay(10000);
  }
  rfSetup();
}

void loop() {

  //if (digitalRead(button0)) {
  //  sensor0Publish();
  //  delay(200); // limit button repetition rate
  //}

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
}
