#include <avr/io.h>
#include <avr/interrupt.h>
#include <SoftwareSerial.h>

SoftwareSerial HC05(12, 13);

#define ENA 11   // OC2A
#define IN1 10   // DDB2
#define IN2 9    // DDB1
#define ENB 3    // OC2B
#define IN3 7    // DDD7
#define IN4 6    // DDD6
#define Encoder_L 2
//pid 변수 20,7/25,9/29,9/30,1,6/
float Pv = 29;
float Iv = 1;
float Dv = 9;
float err_P = 0;
float err_I = 0;
float err_D = 0;
float err_B = 0;
int dutyCycle = 0;  
int goal = 25;
float PID_val = 0;
volatile int estep_L = 0;  
volatile int estep_readL = 0;  

void encoderL(){
  estep_L++;
}

void encoder_read(){
  estep_readL=estep_L;
  estep_L=0;
  Serial.println();
  Serial.print("Encoder value: ");
  Serial.println(estep_readL);
  HC05.print("Encoder value: ");
  HC05.println(estep_readL);
}

void pid_control(){
  err_P= goal-estep_readL;
  err_I+=err_P;
  err_D=err_B-err_P;
  err_B=err_P;
  PID_val=((err_P*Pv)+(err_I*Iv)+(err_D*Dv));
  if(PID_val>=255) PID_val=255;
  if(PID_val<=-255) PID_val=-255;
  dutyCycle = (PID_val+255)/2;
}

//인터럽트
ISR(TIMER1_COMPA_vect) {
  encoder_read();
  pid_control();
  OCR2A = dutyCycle;
  OCR2B = dutyCycle; 
  Serial.print("PID_val: ");
  Serial.println(dutyCycle);
  HC05.print("PID_val: ");
  HC05.println(dutyCycle);
}
// 외부 인터럽트 
ISR(INT0_vect) {
  encoderL();  
}
void setup() {
  Serial.begin(9600);
  HC05.begin(9600);
  DDRB |= (1 << DDB2) | (1 << DDB1) | (1 << DDB3); 
  DDRD |= (1 << DDD3) | (1 << DDD7) | (1 << DDD6); 
  // 타이머2 설정 pwm (핀 11 - OC2A, 핀 3 - OC2B)
  TCCR2A |= (1 << WGM21) | (1 << WGM20) | (1 << COM2A1) | (1 << COM2B1); // Fast PWM 모드3, 비반전 모드 (OC2A, OC2B)
  TCCR2B |= (1 << CS22); // 분주기 64 설정
  PORTB |= (1 << PORTB2); 
  PORTB &= ~(1 << PORTB1); 
  PORTD |= (1 << PORTD7); 
  PORTD &= ~(1 << PORTD6);

  cli();                 // 모든 인터럽트를 비활성화
  TCCR1A = 0;            // 타이머1 제어 레지스터 A 초기화
  TCCR1B = 0;            // 타이머1 제어 레지스터 B 초기화
  TCNT1 = 0;             // 타이머1 카운터 초기화
  OCR1A = 3124;         // 비교 일치 레지스터 값 설정 (0.2초마다 인터럽트 발생)
  TCCR1B |= (1 << WGM12);         // CTC 모드 설정
  TCCR1B |= (1 << CS12) | (1 << CS10); // 1024 분주기 설정
  TIMSK1 |= (1 << OCIE1A);        // 타이머 비교 일치 인터럽트 활성화
  // 외부 인터럽트 설정 (2번 핀, INT0)
  EICRA |= (1 << ISC01);  // Falling edge of INT0 generates an interrupt request
  EIMSK |= (1 << INT0);   // External Interrupt Request 0 Enable
  sei();                 // 모든 인터럽트를 활성화
}
void loop() {
}
