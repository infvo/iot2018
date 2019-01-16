
#include "FS.h"

void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial.println();
  Serial.println("Format file system");

  SPIFFS.format();

  Serial.println("Mount FS");
  if (!SPIFFS.begin()) {
    Serial.println("Failed to mount file system");
    while (1) {};
  } else {
    Serial.println("FS mounted");
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
