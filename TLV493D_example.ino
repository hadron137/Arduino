#include <Wire.h>
#include "TLV493D.h"

TLV493D magnet;
bool sensorsOK = true;
bool initPin = HIGH;

void setup() {
  //pullup should take care of this, but redundancy never hurt
  pinMode(SDA,OUTPUT);
  digitalWrite(SDA,HIGH);
  
  Serial.begin(115200);
  Serial.println("Startup");
  
  delay(100); // give power supply time to come up and stabailze
  Wire.begin();

  unsigned int addressCode = 0x00;
  magnet.generalReset(); // this row needs extra help if power-on is slow

  magnet = TLV493D();
  magnet.begin(initPin);
  magnet.configure(magnet.masterControlled, addressCode);  

}

void loop() {
  int x,y,z;
  sensorsOK = magnet.measure();
  x = magnet.x;
  y = magnet.y;
  z = magnet.z;
}
