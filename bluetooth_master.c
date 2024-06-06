#include <SoftwareSerial.h>

SoftwareSerial HC05(12, 13);

void setup() {
  Serial.begin(9600);
  HC05.begin(9600);
  // put your setup code here, to run once:

}

void loop() {
  HC05.println("gogogo!!");
  HC05.println("good");
  delay(1000);
  // put your main code here, to run repeatedly:

}
