#include "gpsPacket.h"
#include <TinyGPSPlus.h>
#include <Arduino.h>
#include <SoftwareSerial.h>

TinyGPSPlus gps;
SoftwareSerial gpsSerial(8, 6);

void setup() {
  // put your setup code here, to run once:
  //int result = myFunction(2, 3);

  gpsSerial.begin(9600); 
  Serial.begin(9600); 

}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.print("HelloWorld"); 

  while(gpsSerial.available() > 0){
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        float lat = gps.location.lat();
        float lon = gps.location.lng();

        uint8_t buffer[9];
        buildPacket(lat, lon, buffer);
      }
    }
  }
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}