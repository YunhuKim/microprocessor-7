#include <SoftwareSerial.h>

SoftwareSerial HC05(12, 13);

void setup() {
  Serial.begin(9600);
  HC05.begin(9600);
  // put your setup code here, to run once:

}

void loop() {
  if(HC05.available()){
    String text = HC05.readStringUntil(0x0A);
    Serial.println(text);
  
  
  // put your main code here, to run repeatedly:

}
