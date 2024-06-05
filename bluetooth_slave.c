#include <SoftwareSerial.h>

SoftwareSerial HC06(2, 3);

void setup() {
  Serial.begin(9600);
  HC05.begin(9600);
  // put your setup code here, to run once:

}

void loop() {
  if(HC06.available()){
    String text = HC06.readStringUntil(0x0A);
    Serial.println(text);
  
  
  // put your main code here, to run repeatedly:

}