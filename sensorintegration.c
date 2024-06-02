
#define ACCEL_CONFIG      0x1C

#define ACCEL_XOUT_H      0x3B
#define ACCEL_XOUT_L      0x3C
#define ACCEL_YOUT_H      0x3D
#define ACCEL_YOUT_L      0x3E
#define ACCEL_ZOUT_H      0x3F
#define ACCEL_ZOUT_L      0x40


// Set accel scale (+-2g: 0x00, +-4g: 0x01, +-8g: 0x02, +- 16g: 0x03)
uint8_t acc_full_scale = 0x01;

float   acc_scale;
float   ax, ay, az;
char    ax_str[10], ay_str[10], az_str[10];
uint8_t rawData[6];
int16_t accelCount[3];
char    message[255] = {0,};

uint8_t transfer_SPI(uint8_t registerAddress, uint8_t data, bool isRead) {
    uint8_t response = 0x00;
    // Set MSB = 1 for read
    registerAddress |= (isRead ? 0x80 : 0x00);    
    
    // SS: low (active)
    PORTB &= ~(1 << PB2);      
    
    // Register Address transfer
    SPDR = registerAddress;
    while (!(SPSR & (1 << SPIF)));    
    // Data transfer
    SPDR = data;
    while (!(SPSR & (1 << SPIF)));
    response = SPDR;
    
    // SS: high (inactive)
    PORTB |= (1 << PB2);  
    return response;
}





void setup_accel_scale(uint8_t scale) {
  uint8_t current_config = transfer_SPI(ACCEL_CONFIG, 0x00, true);
  // Set 00 to ACCEL_FS_SEL[1:0]
  current_config &= ~0x18;
  // Set ACCEL_FS_SEL to scale
  current_config |= (scale << 3);
  // Write accel config
  transfer_SPI(ACCEL_CONFIG, current_config, false);

  // Set resolution
  switch (scale) {
    case 0x00: // +- 2g
      acc_scale = 2.0f / 32768.0f;
      break;
    case 0x01: // +- 4g
      acc_scale = 4.0f / 32768.0f;
      break;
    case 0x02: // +- 8g
      acc_scale = 8.0f / 32768.0f;
      break;
    case 0x03: // +- 16g
      acc_scale = 16.0f / 32768.0f;
      break;
  }
}

void pressure_sensor(){
  Serial.begin(38400);

  ADMUX |= (0 << REFS1) | (1 << REFS0);
  ADMUX |= (0 << ADLAR);
  ADMUX |= (0 << MUX3) | (0 << MUX2) | (0 << MUX1) | (0 << MUX0);
  ADCSRA |= (1 << ADEN);
  ADCSRA |= (1 << ADPS2) | (1<< ADPS1) | (1<< ADPS0);
}



void setup() {
  Serial.begin(38400);

  ////////////////////////////////////////////////////////////
  // Set data direction
  // - PORTB Output: PB2 (SS), PB3 (COPI), PB5 (SCK)
  // - PORTB Input: PB4 (CIPO)
  // => DDRB = 0b XX10 11XX
  ////////////////////////////////////////////////////////////
  DDRB |=  (1 << PB0) | (1 << PB2) | (1 << PB3) | (1 << PB5);
  DDRB &= ~(1 << PB4);

  ////////////////////////////////////////////////////////////
  // Set registers for SPI communication
  // - SPCR = 0b 
  //  >> (1) SPE (SPI Enable): SPI is enabled
  //  >> (0) DORD (Data Order): MSB first
  //  >> (1) MSTR (Master/Slave select): Set as SPI Master
  //  >> (0) CPOL (Clock Polarity): Latch data on rising edge
  //  >> (0) CPHA (Clock Phase): Data transfer on the falling edge
  //  >> (01) SPR[1:0] (SPI Clock Rate): Max. CLK freq. of IMU is 1MHz
  ////////////////////////////////////////////////////////////
  SPCR |=   (1 << SPE)  | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
  SPCR &= ~((1 << DORD) | (1 << CPOL) | (1 << CPHA));

  // Initialize
  // SS: high (inactive)
  PORTB |= (1 << PB2);  
  



  // Set accel scale
  setup_accel_scale(acc_full_scale);
  pressure_sensor();

  delay(1000);
}

void loop() {
  // Get raw data from accelerometer
  rawData[0] = transfer_SPI(ACCEL_XOUT_H, 0x00, true);
  rawData[1] = transfer_SPI(ACCEL_XOUT_L, 0x00, true);
  rawData[2] = transfer_SPI(ACCEL_YOUT_H, 0x00, true);
  rawData[3] = transfer_SPI(ACCEL_YOUT_L, 0x00, true);
  rawData[4] = transfer_SPI(ACCEL_ZOUT_H, 0x00, true);
  rawData[5] = transfer_SPI(ACCEL_ZOUT_L, 0x00, true);

  accelCount[0] = (rawData[0] << 8) | rawData[1];
  accelCount[1] = (rawData[2] << 8) | rawData[3];
  accelCount[2] = (rawData[4] << 8) | rawData[5];

  // Scaling 
  ax = accelCount[0] * acc_scale;
  ay = accelCount[1] * acc_scale;
  az = accelCount[2] * acc_scale;

  dtostrf(ax, 4, 2, ax_str);
  dtostrf(ay, 4, 2, ay_str);
  dtostrf(az, 4, 2, az_str);

while(ADCSRA & (1 << ADSC));
  ADCSRA |= (1 << ADSC);
  uint16_t value = ADC;
  Serial.println(value);
  delay(1000);




  sprintf(message, "ax = %s, ay = %s, az = %s", ax_str, ay_str, az_str);
  Serial.println(message);

  if(ay > 0.50 & value > 100){
  PORTB |= (1 << PB0);  // PB0 핀을 HIGH로 설정
} else {
  PORTB &= ~(1 << PB0);  // PB0 핀을 LOW로 설정
}

delay(200);  // 딜레이 설정



}