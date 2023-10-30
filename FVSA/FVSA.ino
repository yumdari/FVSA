/********************Front Vehicle Start Alarm********************/
/*Alerts the driver when the vehicle in front has started to move*/
/*2023년도 ICT이노베이션스퀘어 확산 사업 - 아두이노 기반 모빌리티 IoT 과정*/
//  Changelog:
//  23.10.30 - create varibles for detecting car
//  23.10.29 - create project
//           - ultrasonic sensor setting

#define TRIG 5
#define ECHO 6
#define BUZZER 11

enum carState {
  STOP = 0,
  DEPART
};

typedef struct {
  uint8_t frontCarState;
  uint8_t myCarState;
}CarState;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  // 초음파센서의 동작 상태를 확인하기 위해 시리얼 통신 설정(전송속도 9600bps)
  
  pinMode(TRIG, OUTPUT);  //초음파 송신부를 출력 설정
  pinMode(ECHO, INPUT);   //초음파 수신부를 입력 설정
  
  pinMode(BUZZER, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); 
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW); // 10ms만큼 출력

  unsigned long duration = pulseIn(ECHO, HIGH); // ECHO 핀이 HIGH 상태가 될 때까지 시간 측정

  float distance = ((float)(340*duration) / 10000) / 2; // 초음파는 1초당 340m를 이동
                                          // 따라서, 초음파의 이동 거리 = duration(왕복에 걸린시간)*340 / 1000 / 2

  Serial.print(distance);   // 측정된 거리 값를 시리얼 모니터에 출력
  Serial.println("cm");
  
  delay(1000);
}
