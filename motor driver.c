#define EA    3   // EA ------- D3 
#define EB    11  // EB ------- D11
#define M_IN1 4   // IN1 ------- D4
#define M_IN2 5   // IN2 ------- D5
#define M_IN3 13  // IN3 ------- D13
#define M_IN4 12  // IN4 ------- D12

int motorA_vector = 1; // 모터의 회전방향이 반대일 시 0을 1로
                       // 1을 0으로 바꿔주시면 모터의 회전방향이 바뀜.

int motorB_vector = 1; // 모터의 회전방향이 반대일 시 0을 1로
                       // 1을 0으로 바꿔주시면 모터의 회전방향이 바뀜.

void setup() { // 초기화
  // 모터드라이버와 연결된 핀 출력설정
  pinMode(EA, OUTPUT);
  pinMode(EB, OUTPUT);
  pinMode(M_IN1, OUTPUT);
  pinMode(M_IN2, OUTPUT);
  pinMode(M_IN3, OUTPUT);
  pinMode(M_IN4, OUTPUT);
}

void loop() { // 무한루프
  // DC모터의 특성상 극성의 방향을 바꿔줌으로써
  // 정회전, 역회전을 출력할 수 있다.
  
  // 모터 A 정회전
  digitalWrite(EA, HIGH);         // MotorA 출력 On
  digitalWrite(M_IN1, motorA_vector);
  digitalWrite(M_IN2, !motorA_vector);

  // 모터 B 정회전
  digitalWrite(EB, HIGH);         // MotorB 출력 On
  digitalWrite(M_IN3, motorB_vector);
  digitalWrite(M_IN4, !motorB_vector);

  delay(5000);                     // 딜레이 5초 (5초간 정회전)
}


