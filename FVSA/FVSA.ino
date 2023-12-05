/********************Front Vehicle Start Alarm********************/
/*Alerts the driver when the vehicle in front has started to move*/
/*2023년도 ICT이노베이션스퀘어 확산 사업 - 아두이노 기반 모빌리티 IoT 과정*/
//  Changelog:
//  23.12.05 - modify variables to fit camel notation (predistance -> preDistance)
//           - add LCD function
//  23.11.05 - front car detecting logic
//             distnace filtering logic
//             divide function 
//  23.10.30 - create varibles for detecting car
//  23.10.29 - create project
//             ultrasonic sensor setting

#include <Wire.h>               // I2C 라이브러리
#include <LiquidCrystal_I2C.h>  // I2C를 사용한 LCD 라이브러리

#define TRIG 5
#define ECHO 6
#define ULTRA_DELAY 1000

#define BUZZER 11

LiquidCrystal_I2C lcd(0x27, 16, 2); // LCD 객체 선언

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

CarState myCarState;
CarState frontCarState;

unsigned long curTime = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  // 초음파센서의 동작 상태를 확인하기 위해 시리얼 통신 설정(전송속도 9600bps)

  lcd.begin();  //  LCD 사용
  
  pinMode(TRIG, OUTPUT);  //초음파 송신부를 출력 설정
  pinMode(ECHO, INPUT);   //초음파 수신부를 입력 설정
  
  pinMode(BUZZER, OUTPUT);

  memset(&myCarState, 0x00, sizeof(myCarState));
  memset(&frontCarState, 0x00, sizeof(frontCarState));
  
  curTime = millis();  
}

void loop() {
  // put your main code here, to run repeatedly:

  float distance = UltraSonic();
  float* pDistance = &distance;
   
#ifdef FILTER
  DistanceFilter(pDistance);
#endif

  Serial.print(distance);   // 측정된 거리 값를 시리얼 모니터에 출력
  Serial.println("cm");
  
  Lcd(distance);
  
  DetectCar(pDistance);

  delay(ULTRA_DELAY);
}

void Lcd(float dist)
{
  lcd.clear();
  lcd.setCursor(0, 0);    // 커서를 0, 0에 위치 (열, 행)
  lcd.print("distance : ");     // 0, 0에 distance를 출력

  lcd.setCursor(11, 0);    // 커서를 0, 0에 위치 (열, 행)
  
  if(dist > 0)
  {
      lcd.print(dist);
  }
  else
  lcd.print("error"); //  out of lange
}

float UltraSonic()
{
  float ret = 0;
  
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); 
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW); // 10ms만큼 출력

  unsigned long duration = pulseIn(ECHO, HIGH); // ECHO 핀이 HIGH 상태가 될 때까지 시간 측정

  if((((float)(340*duration) / 10000) / 2 >= 2)&&(((float)(340*duration) / 10000) / 2 <= 400))
    ret = ((float)(340*duration) / 10000) / 2;      // 초음파는 1초당 340m를 이동
                                                    // 따라서, 초음파의 이동 거리 = duration(왕복에 걸린시간)*340 / 1000 / 2
  return ret;
}

void DetectCar(float *pd)
{
  switch(frontCarState.stopStart)
  {
    case INVALID :
    if((*pd <= 30))
    {
      frontCarState.stopStart = DETECTED;
      Serial.println("front car detected");
    }
    break;
    
    case DETECTED : 
    if(millis() > curTime + 5000)
    {
      if(*pd <= 30)
      {
        frontCarState.stopStart = STOP;
        Serial.println("front car stopped");
        curTime = millis();
      }
      else
        frontCarState.stopStart = INVALID;
    }
    break;
  
    case STOP :
    if(millis() > curTime + 3000)
    {
      if(*pd > 30)
      {
        frontCarState.stopStart = DEPART;
        Serial.println("front car departed");
        digitalWrite(11, HIGH);
        curTime = millis();
      }
    }
    break;
    
    case DEPART : 
      frontCarState.stopStart = INVALID;
      Serial.println("front car status init");
      digitalWrite(11, LOW);
    break;
  }
}

void DistanceFilter(float *pd)
{
  static float preDistance = 0;

  if(preDistance != 0)
  {
    if ((*pd>(preDistance*10)) || (*pd < (preDistance/15))) // 초음파 센서 측정값 튀는걸 필터링
      {
        if((*pd>(preDistance*10)))
          Serial.println("filtering *");
        else 
          Serial.println("filtering /");
        *pd = preDistance;
      }
  }
  else
  preDistance = *pd;
}
