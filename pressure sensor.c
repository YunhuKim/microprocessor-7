void setup(){
  Serial.begin(9600);

  ADMUX |= (0 << REFS1) | (1 << REFS0);
  ADMUX |= (0 << ADLAR);
  ADMUX |= (0 << MUX3) | (0 << MUX2) | (0 << MUX1) | (0 << MUX0);
  ADCSRA |= (1 << ADEN);
  ADCSRA |= (1 << ADPS2) | (1<< ADPS1) | (1<< ADPS0);
}

void loop(){
  while(ADCSRA & (1 << ADSC));
  ADCSRA |= (1 << ADSC);
  uint16_t value = ADC;
  Serial.println(value);
  delay(1000);



}
