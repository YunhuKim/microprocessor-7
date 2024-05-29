#define ENA 11   
#define IN1 10   
#define IN2 9   

#define ENB 6    
#define IN3 7    
#define IN4 8    

volatile uint8_t dutyCycleA = 200;
volatile uint8_t dutyCycleB = 200;

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  TCCR1A |= (1 << WGM11) | (1 << COM1A1);
  TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS12);
  ICR1 = 255;

  TCCR2A |= (1 << WGM21) | (1 << WGM20) | (1 << COM2A1);
  TCCR2B |= (1 << CS22);

  analogWrite(ENA, dutyCycleA);
  analogWrite(ENB, dutyCycleB);
}

void loop() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  Serial.print("Motor A Duty Cycle: ");
  Serial.println(dutyCycleA);
  Serial.print("Motor B Duty Cycle: ");
  Serial.println(dutyCycleB);

  delay(1000);
}
