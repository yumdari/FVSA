/********************Front Vehicle Start Alarm********************/
/*Alerts the driver when the vehicle in front has started to move*/
/*2023년도 ICT이노베이션스퀘어 확산 사업 - 아두이노 기반 모빌리티 IoT 과정*/
//  Changelog:
//  23.12.20 - remove joystick function
//  23.12.10 - remove main loop delay (ULTRA_DELAY)
//           - remove LCD Clear
//           - add init time(2secs) for distance value OVF
//  23.12.07 - add UART Define (Ultrasonic, Detect_car)
//           - add Joystic Function
//  23.12.06 - add LED Indication
//           - modify detect car logic
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

#define TRIG 5            //  초음파센서 송신부
#define ECHO 6            //  초음파센서 수신부
#define ULTRA_DELAY 1000  // 초음파센서 측정 간격 1초
#define LCD_DELAY 2000

#define BUZZER  11        // 부저

#define RED   2           // 적색 LED
#define GREEN 3           // 녹색 LED
#define BLUE  4           // 청색 LED

//#define UART_ULTRA      // 초음파센서 함수 디버깅용
//#define UART_DETECT_CAR // 차량 감지 함수 디버깅용
//#define FILTER          // 초음파센서 필터링

LiquidCrystal_I2C lcd(0x27, 16, 2); // LCD 객체 선언

typedef struct{           // 전방 차량 상태 저장용 구조체
  uint8_t stopStart;
  uint8_t preStopStart;
  float lastDistance;
}CarState;
CarState frontCarState;

typedef struct {            // LCD에 표시할 거리를 저장하는 구조체
  float distance;
}LcdData;
LcdData lcdData;

enum carState {           // 전방 차량 상태 표시용 열거형
  INVALID = 0,
  DETECTED,
  STOP,
  DEPART
};

enum lcdCmd {           // LCD 처리용 열거형
  DISTANCE = 0,
  STATE
};

unsigned long curTimeDetectCar;   // 차량 감지용 시간 변수
unsigned long curTimeUltrasonic;  // 초음파센서용 시간 변수

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);       // 초음파센서의 동작 상태를 확인하기 위해 시리얼 통신 설정(전송속도 9600bps)

  lcd.begin();              //  LCD 사용
  
  pinMode(TRIG, OUTPUT);    //  초음파 송신부를 출력 설정
  pinMode(ECHO, INPUT);     //  초음파 수신부를 입력 설정
  
  pinMode(BUZZER, OUTPUT);  //  부저를 출력 설정

  pinMode(RED, OUTPUT);     //  LED 출력 설정
  pinMode(GREEN, OUTPUT);   //  LED 출력 설정
  pinMode(BLUE, OUTPUT);    //  LED 출력 설정

  memset(&frontCarState, 0x00, sizeof(frontCarState));  // 차량 감지 구조체 초기화
  memset(&lcdData, 0x00, sizeof(lcdData));              // LCD 데이터 구조체 초기화

  curTimeDetectCar = millis();    // 차량 감지 시간 변수 초기화
  curTimeUltrasonic = millis();   // 초음파센서 시간 변수 초기화

  Serial.print("system init");
}

void loop() {
  // put your main code here, to run repeatedly:
  
  float distance = UltraSonic();  // 초음파센서로 거리 측정
  float* pDistance = &distance;   // distance의 주소를 담는 포인터 변수
  
#ifdef FILTER
  DistanceFilter(pDistance);      // 측정 거리 필터링
#endif

  Lcd(DISTANCE);                  // LCD에 거리 표시
  
  DetectCar(distance);            // 차량 감지
  
} 

void Led(int color)
{
  digitalWrite(RED, LOW);         // LED 모두 끔
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, LOW);

  if(frontCarState.stopStart != INVALID)  // 차량 감지된 상태일 때, 넘겨받은 색상 출력
  {
    digitalWrite(color, HIGH);
  }
}

void Lcd(uint8_t cmd)
{
      lcd.setCursor(0, 0);        // 커서를 0, 0에 위치 (열, 행)
      lcd.print("distance : ");   // 0, 0에 distance를 출력
      lcd.setCursor(0, 1);        // 커서를 0, 1에 위치 (열, 행)
      lcd.print("state : ");      // 1, 0에 state 출력
  switch (cmd)
  {
    case DISTANCE:    
      lcd.setCursor(11, 0);       // 커서를 0, 0에 위치 (열, 행)
    
      if(millis() > (LCD_DELAY))  // 초음파 센서 딜레이 1초 이전에 거리 값 ovf 출력 방지
      {
        if(lcdData.distance > 0)  // 거리가 0보다 크면 LCD에 거리 출력
        {
            lcd.print(lcdData.distance);
            if (lcdData.distance < 10)
            { 
              lcd.setCursor(15, 0);
              lcd.print(" ");
            }
        }
        else
        lcd.print("error");       // out of lange
      }
      else
      {
        lcd.print("init");       // 초기화 시간 2초 
        lcd.setCursor(8, 1);     // 커서를 11, 0에 위치 (열, 행) 
        lcd.print("INVALID ");   // 8, 1에 INVALID 출력
      }
    
    break;

    case STATE:
    lcd.setCursor(8, 1);      // 커서를 8, 0에 위치 (열, 행) 
    switch (frontCarState.stopStart)  // 전방 차량 상태에 따라 LCD에 상태 출력
    {
      case INVALID :
      lcd.print("INVALID ");   // 8, 1에 INVALID 출력
      break;
      
      case DETECTED : 
      lcd.print("DETECTED");   // 8, 1에 DETECTED 출력
      break;
    
      case STOP :
      lcd.print("STOP    ");   // 8, 1에 STOP 출력
      break;
          
      case DEPART : 
      lcd.print("DEPART  ");   // 8, 1에 DEPART 출력
      break;    
    }
  }
}

float UltraSonic()
{
  static float ret = 0;

  if(millis() >= curTimeUltrasonic + ULTRA_DELAY) // 1초(ULTRA_DELAY) 지났을 경우 실행
  {
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH); 
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW); // 10ms만큼 출력
  
    unsigned long duration = pulseIn(ECHO, HIGH);     // ECHO 핀이 HIGH 상태가 될 때까지 시간 측정
  
    if((((float)(340*duration) / 10000) / 2 >= 2) && (((float)(340*duration) / 10000) / 2 <= 400))
    {
      ret = ((float)(340*duration) / 10000) / 2;      // 초음파는 1초당 340m를 이동
                                                      // 따라서, 초음파의 이동 거리 = duration(왕복에 걸린시간)*340 / 1000 / 2
    }
#ifdef UART_ULTRA
    Serial.print(ret);   // 측정된 거리 값를 시리얼 모니터에 출력
    Serial.println("cm");
#endif
    curTimeUltrasonic = millis(); // 초음파센서 시간 변수 초기화
    lcdData.distance = ret;       // LCD에 표시할 거리 변수 저장
    return ret;                   // 거리값을 반환
  }
}

void DetectCar(float distance)
{
  switch(frontCarState.stopStart)          // 전방 차량 상태에 따라 실행
  {
    case INVALID :                         // 아무 것도 아닌 상태일 때
    if((distance <= 30) && (distance > 0)) // 거리가 30cm보다 작으면
    {
      frontCarState.stopStart = DETECTED;  // '감지' 상태로 변경하고
#ifdef UART_DETECT_CAR
      Serial.println("front car detected");
#endif
      Led(GREEN);                          // 녹색 LED 켬
      Lcd(STATE);                          // LCD에 표시
      curTimeDetectCar = millis();
    }
    break;
    
    case DETECTED :                          // '감지' 상태일 때 
    if(millis() >= curTimeDetectCar + 4000)  // 4초가 지나고
    {
      if((distance <= 30) && (distance > 0)) // 거리가 30cm보다 작으면
      {
        frontCarState.stopStart = STOP;      // '정지' 상태로 변경하고
#ifdef UART_DETECT_CAR
        Serial.println("front car stopped");
#endif
        frontCarState.lastDistance = distance;
        Led(RED);                           //적색 LED 켬
        Lcd(STATE);                         // LCD에 표시
        curTimeDetectCar = millis();
        break;
      }
    }
    if(distance > 30)                       // 4초가 지나기 전에 전방 차량과의 거리가 30cm 초과하면 
    {
        frontCarState.stopStart = INVALID;  // 차량 상태를 초기화하고
#ifdef UART_DETECT_CAR
        Serial.println("front car status init");
#endif
        Led(INVALID);                      // LED 끔
        Lcd(STATE);                        // LCD에 표시
        curTimeDetectCar = millis();
    }
    break;
  
    case STOP :                                 // '정지' 상태일 때
    if(millis() >= curTimeDetectCar + 2000)     // 2초가 지나고
    {
      if((distance > 30) && (distance > 0))     // 앞차와의 거리가 30cm 초과라면
      {
        frontCarState.stopStart = DEPART;       // 차량 상태를 '출발'로 변경하고
#ifdef UART_DETECT_CAR
        Serial.println("front car departed");
#endif
        digitalWrite(BUZZER, HIGH);             // 부저로 알림
        Led(BLUE);                              // 청색 LED 켬
        Lcd(STATE);                             // LCD에 표시
        curTimeDetectCar = millis();
        break;
      }
      else if((distance <= 30) && (distance > 0))
      {
        /*전방 차량 멈춰 있음*/
      }
    }
    if(distance > 30)                           // 2초가 지나기 전에 앞차와의 거리가 30cm 초과하면
    {
        frontCarState.stopStart = INVALID;      // 차량 상태를 초기화
#ifdef UART_DETECT_CAR
        Serial.println("front car status init");
#endif
        Led(INVALID);                           // LED 끔
        Lcd(STATE);                             // LCD에 표시
        curTimeDetectCar = millis();
    }
    break;
     
    case DEPART :                               // 차량 상태가 '출발'일 때
    if(millis() >= curTimeDetectCar + 1000)     // 1초가 지났다면
    {
      frontCarState.stopStart = INVALID;        // 차량 상태를 초기화하고
#ifdef UART_DETECT_CAR
      Serial.println("front car status init");
#endif
      digitalWrite(BUZZER, LOW);                // 부저 끔
      Led(INVALID);                             // LED 끔
      Lcd(STATE);                               // LCD에 표시
      curTimeDetectCar = millis();
      break;
    }
  }
}

void DistanceFilter(float *pd)
{
  static float preDistance = 0;

  if(preDistance != 0)
  {
    if ((*pd > (preDistance * 10)) || (*pd < (preDistance / 15))) // 초음파 센서 측정값 튀는걸 필터링 (이전 값보다 10배 초과 또는 1/15배보다 작은경우)
      {
        if((*pd > (preDistance * 10)))
        {
#ifdef UART_DETECT_CAR
          Serial.println("filtering *");
#endif
        }
        else 
        {
#ifdef UART_DETECT_CAR
          Serial.println("filtering /");
#endif
        }
        *pd = preDistance;  // 현재 측정 값에 문제가 있을 경우, 무시하고 이전 값으로 저장
      }
  }
  else
  preDistance = *pd;        // 현재 측정 값에 이상 없을 경우 이전 값에 현재 측정 값을 저장
}
