#include "SystemStatus.h"
SystemStatus sys;
void setup(){
  Serial.begin(9600);
}
void loop(){
  Serial.println(sys.getVCC());
  delay(1000);
}