/********************Front Vehicle Start Alarm********************/
/*Alerts the driver when the vehicle in front has started to move*/
/*2023년도 ICT이노베이션스퀘어 확산 사업 - 아두이노 기반 모빌리티 IoT 과정*/
//  Changelog:
//  23.11.05 - front car detecting logic
//  23.10.30 - create varibles for detecting car
//  23.10.29 - create project
//           - ultrasonic sensor setting

#define TRIG 5
#define ECHO 6
#define ULTRA_DELAY 1000

#define BUZZER 11

enum carState {
  INVALID = 0,
  DETECTED,
  STOP,
  DEPART
};

typedef struct {
  uint8_t stopStart;
  uint8_t preStopStart;
}CarState;

unsigned long curTime = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  // 초음파센서의 동작 상태를 확인하기 위해 시리얼 통신 설정(전송속도 9600bps)
  
  pinMode(TRIG, OUTPUT);  //초음파 송신부를 출력 설정
  pinMode(ECHO, INPUT);   //초음파 수신부를 입력 설정
  
  pinMode(BUZZER, OUTPUT);

  curTime = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  CarState myCarState;
  CarState frontCarState;

  memset(&myCarState, 0x00, sizeof(myCarState));
  memset(&frontCarState, 0x00, sizeof(frontCarState));


  static uint8_t alert = 0;
  
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); 
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW); // 10ms만큼 출력

  unsigned long duration = pulseIn(ECHO, HIGH); // ECHO 핀이 HIGH 상태가 될 때까지 시간 측정

  static float distance = 0;
  if((duration >= 117.6471)&&(duration <= 23529.41176))
    distance = ((float)(340*duration) / 10000) / 2; // 초음파는 1초당 340m를 이동
                                                        // 따라서, 초음파의 이동 거리 = duration(왕복에 걸린시간)*340 / 1000 / 2
  static float preDistance = 0;
  
  if(preDistance == 0)
  {
    preDistance = distance;
    Serial.println("preDistance init");
  }

  if(preDistance != 0)
  {
    if ((distance>(preDistance*10)) || (distance < (preDistance/15))) // 초음파 센서 측정값 튀는걸 필터링
      {
        if((distance>(preDistance*5)))
          Serial.println("filtering *");
        else 
          Serial.println("filtering /");
       distance = preDistance;
      }
  }

  Serial.print(distance);   // 측정된 거리 값를 시리얼 모니터에 출력
  Serial.println("cm");

  switch(frontCarState.stopStart)
  {
    case INVALID :
    if(distance <= 30)
    {
      frontCarState.stopStart = DETECTED;
      Serial.println("front car detected");
    }
    break;
    
    case DETECTED : 
    if(millis() > curTime + 3000)
    {
      if(distance <= 30)
      {
        frontCarState.stopStart = STOP;
        Serial.println("front car stopped");
        curTime = millis();
      }
    }
    break;
  
    case STOP :
    if(millis() > curTime + 3000)
    {
      if(distance > 30)
      {
        frontCarState.stopStart = DEPART;
        Serial.println("front car departed");
        curTime = millis();
      }
    }
    break;
    
    case DEPART : 
     //digitalWrite(13, HIGH);
    Serial.println("alerted");
    break;
  }

  delay(ULTRA_DELAY);
  preDistance  = distance;
}
