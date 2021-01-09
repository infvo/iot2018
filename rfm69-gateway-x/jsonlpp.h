#include <ArduinoJson.h>
#include <string>

// omzetten van JSON in Cayenne-formaat; en omgekeerd.
// voor het ontleden van de JSON-string gebruiken we de Arduino JSON library


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


// Data ID + Data Type + Data Size
#define LPP_DIGITAL_INPUT_SIZE       3
#define LPP_DIGITAL_OUTPUT_SIZE      3
#define LPP_ANALOG_INPUT_SIZE        4
#define LPP_ANALOG_OUTPUT_SIZE       4
#define LPP_LUMINOSITY_SIZE          4
#define LPP_PRESENCE_SIZE            3
#define LPP_TEMPERATURE_SIZE         4
#define LPP_RELATIVE_HUMIDITY_SIZE   3
#define LPP_ACCELEROMETER_SIZE       8
#define LPP_BAROMETRIC_PRESSURE_SIZE 4
#define LPP_GYROMETER_SIZE           8
#define LPP_GPS_SIZE                 11

class LppInput {
    uint8_t *buffer;
    unsigned int index = 0;
    unsigned int length = 0;
    
    uint8_t getByte();
    uint16_t getUint();
    int16_t getInt();
    bool endOfBuffer();
    void toJsonItem(JsonObject& item);
    
  public:    
    LppInput(uint8_t buffer[], unsigned int length);
    void toJson (JsonObject& payload);   
};

LppInput::LppInput(uint8_t buf[], unsigned int len) {
  buffer = buf;
  length = len;
  index = 0;
}

uint8_t LppInput::getByte() {
  return buffer[index++];
}

uint16_t LppInput::getUint() {
  uint8_t hiByte = getByte();
  return (hiByte <<8) | getByte(); 
}

int16_t LppInput::getInt() {
  uint8_t hiByte = getByte();
  return (int) ((hiByte <<8) | getByte());
}

bool LppInput::endOfBuffer() {
  return index >= length;
}

void LppInput::toJsonItem(JsonObject& payload) {
  
  unsigned int channelNr = getByte(); // e.g., 12
  JsonObject& object1 = payload.createNestedObject(String(channelNr)); 
  
  unsigned int typeTag = getByte();
  String s1;
  switch (typeTag) {
    case LPP_DIGITAL_INPUT:
      object1["dIn"] = getByte();
      break;
    case LPP_DIGITAL_OUTPUT:
      object1["dOut"] = getByte();
      break;  
    case LPP_ANALOG_INPUT:
       object1["aIn"] = getUint(); 
      break;
    case LPP_ANALOG_OUTPUT:
       object1["aOut"] = getUint(); 
      break;     
    case LPP_TEMPERATURE :
       object1["temperature"] = getInt(); 
      break;
    case LPP_BAROMETRIC_PRESSURE:
       object1["barometer"] = getInt(); 
      break;
    case LPP_RELATIVE_HUMIDITY:
       object1["humidity"] = getByte();
      break;                   
    default:
       object1["error"] = typeTag;
  }
}

void LppInput::toJson (JsonObject& payload) {

  index = 0;
  while (!endOfBuffer()) {
    toJsonItem(payload);
  }
}

void lppToJson(uint8_t lpp[], unsigned int len, JsonObject& root) {
  LppInput lppIn(lpp, len);
  lppIn.toJson(root);
}

String lppToJsonString(uint8_t lpp[], unsigned int len) {
  StaticJsonBuffer<512> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  
  lppToJson(lpp, len, root);
  
  String s;
  root.printTo(s);
  return s;
}

int jsonToLpp(JsonObject& root, uint8_t lpp[]) {
  int index = 0;
  int port = 1; // default port
  
  if (root["port"]) {
    port = root["port"];
    if (port < 256 && port > 0) {
      lpp[index++] = port;
    } else {
     Serial.println("Illegal port, ignored");
     return -1;
    }
  } else {
    Serial.println("port missing, assuming 1");
  }
  lpp[index++] = port;

//  unsigned long nodeid = 0;
//  if (root["nodeid"]) {
//    JsonVariant nodeidVariant = root["nodeid"];
//    if (nodeidVariant.is<unsigned int>()) {
//      nodeid = nodeidVariant.as<unsigned int>();
//    } else if (nodeidVariant.is<char*>()) {
//      String nodeidString = nodeidVariant.as<char*>();
//      char *ptr;
//      nodeid = strtol(nodeidString.c_str(), &ptr, 16);
//      if (*ptr != '\0' || nodeid >= 65536L || nodeid == 0) {
//        Serial.println("nodeid incorrect");
//        return -1;
//      }
//    } else {
//      Serial.println("nodeid incorrect");
//      return -1;
//    }
//  } else {
//    Serial.println("nodeid missing");
//    return -1;
//  }
//  lpp[index++] = highByte(nodeid);
//  lpp[index++] = lowByte(nodeid);

  for (JsonPair& p : root) {  // for all p: p.key, p.value
  
    char *ptr;  // see: https://stackoverflow.com/questions/19148611/using-strtol-to-validate-integer-input-in-ansi-c
    long key = strtol(p.key, &ptr, 10); // convert key to int (if possible)
    if (*ptr == '\0') {                 // valid int found
      lpp[index++] = (uint8_t) key;     // channel nr.
      
      JsonObject& obj = p.value;
      if (obj.containsKey("dOut")) {
        int dValue = obj["dOut"].as<int>();
        lpp[index++] = dValue;  
      } else if (obj.containsKey("aOut")) {
        int aValue = obj["aOut"].as<int>();
        lpp[index++] = highByte(aValue);
        lpp[index++] = lowByte(aValue);       
      } else {
        // Serial.println("no valid output type");
        // just skip - 
      }
    } else {      
      // Serial.println(p.key); // is a const char* pointing to the key 
      // skip   --  p.value is a JsonVariant
    }   
  }
  lpp[index++] = (uint8_t) 0xff;  
  return index;
}
