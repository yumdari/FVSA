/********************Front Vehicle Start Alarm********************/
/*Alerts the driver when the vehicle in front has started to move*/
/*2023년도 ICT이노베이션스퀘어 확산 사업 - 아두이노 기반 모빌리티 IoT 과정*/
//  Changelog:
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

#define TRIG 5
#define ECHO 6
#define ULTRA_DELAY 1000
#define LCD_DELAY 2000

#define BUZZER  11

#define JOY_SW  7
#define JOY_X   A1
#define JOY_Y   A0

#define RED   2
#define GREEN 3
#define BLUE  4

//#define UART_ULTRA
//#define UART_DETECT_CAR
//#define FILTER

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
  float lastDistance;
}CarState;

CarState frontCarState;

typedef struct {
  uint8_t cmd;
  uint8_t data;
  float distance;
}LcdMsg;

LcdMsg lcdMsg;

enum lcdCmd {
  DISTANCE = 0,
  JOYSTICK
};


unsigned long curTimeDetectCar;
unsigned long curTimeUltrasonic;
unsigned long curTimeJoyStick;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);     // 초음파센서의 동작 상태를 확인하기 위해 시리얼 통신 설정(전송속도 9600bps)

  lcd.begin();            //  LCD 사용
  
  pinMode(TRIG, OUTPUT);  //초음파 송신부를 출력 설정
  pinMode(ECHO, INPUT);   //초음파 수신부를 입력 설정
  
  pinMode(BUZZER, OUTPUT);

  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  pinMode(JOY_SW, INPUT_PULLUP);  // 조이스틱 버튼에 내부 pull up 적용

  memset(&frontCarState, 0x00, sizeof(frontCarState));

  curTimeDetectCar = millis();
  curTimeUltrasonic = millis();
  curTimeJoyStick = millis();
  Serial.print("system init");
}

void loop() {
  // put your main code here, to run repeatedly:
  
  float distance = UltraSonic();
  float* pDistance = &distance;
  
#ifdef FILTER
  DistanceFilter(pDistance);
#endif

  Lcd(DISTANCE);
  
  DetectCar(distance);
  
  Joystick();
}

void Joystick()
{
  Serial.print("X : ");
  Serial.println(analogRead(JOY_X));
 
  Serial.print("Y : ");
  Serial.println(analogRead(JOY_Y));

  Serial.print("Button : ");
  Serial.println(digitalRead(JOY_SW));
}

void Led(int color)
{
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, LOW);

  if(frontCarState.stopStart != INVALID)
  {
    digitalWrite(color, HIGH);
  }
}

void Lcd(uint8_t cmd)
{
  switch (cmd)
  {
    case DISTANCE:
    lcd.setCursor(0, 0);        // 커서를 0, 0에 위치 (열, 행)
    lcd.print("distance : ");   // 0, 0에 distance를 출력
    
    lcd.setCursor(11, 0);       // 커서를 0, 0에 위치 (열, 행)
  
    if(millis() > (LCD_DELAY))  // 초음파 센서 딜레이 1초 이전에 거리 값 ovf 출력 방지
    {
      if(lcdMsg.distance > 0)
      {
          lcd.print(lcdMsg.distance);
          if (lcdMsg.distance < 10)
          { 
            lcd.setCursor(15, 0);
            lcd.print(" ");
          }
      }
      else
      lcd.print("error");       // out of lange
    }
    else
    lcd.print("init");          // 초기화 시간 2초 
  }
}

float UltraSonic()
{
  static float ret = 0;

  if(millis() >= curTimeUltrasonic + ULTRA_DELAY)
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
    curTimeUltrasonic = millis();
    lcdMsg.distance = ret;
    return ret;
  }
}

void DetectCar(float distance)
{
  switch(frontCarState.stopStart)
  {
    case INVALID :
    if((distance <= 30) && (distance > 0))
    {
      frontCarState.stopStart = DETECTED;
#ifdef UART_DETECT_CAR
      Serial.println("front car detected");
#endif
      Led(GREEN);
    }
    break;
    
    case DETECTED : 
    if(millis() >= curTimeDetectCar + 5000)
    {
      if((distance <= 30) && (distance > 0))
      {
        frontCarState.stopStart = STOP;
#ifdef UART_DETECT_CAR
        Serial.println("front car stopped");
#endif
        frontCarState.lastDistance = distance;
        curTimeDetectCar = millis();
        Led(RED);
      }
      else
      {
        frontCarState.stopStart = INVALID;
#ifdef UART_DETECT_CAR
        Serial.println("front car status init");
#endif
        Led(INVALID);
      }
    }
    break;
  
    case STOP :
    if(millis() >= curTimeDetectCar + 3000)
    {
      if((distance > 30) && (distance > 0))
      {
        frontCarState.stopStart = DEPART;
#ifdef UART_DETECT_CAR
        Serial.println("front car departed");
#endif
        digitalWrite(BUZZER, HIGH);
        curTimeDetectCar = millis();
        Led(BLUE);
      }
      else if((distance <= 30) && (distance > 0))
      {
        /*전방 차량 멈춰 있음*/
      }
      else
      {
        frontCarState.stopStart = INVALID;
#ifdef UART_DETECT_CAR
        Serial.println("front car status init");
#endif
        Led(INVALID);
      }
    }
    break;    
    case DEPART : 
      frontCarState.stopStart = INVALID;
#ifdef UART_DETECT_CAR
      Serial.println("front car status init");
#endif
      digitalWrite(BUZZER, LOW);
      Led(INVALID);
      break;
  }
}

void DistanceFilter(float *pd)
{
  static float preDistance = 0;

  if(preDistance != 0)
  {
    if ((*pd > (preDistance * 10)) || (*pd < (preDistance / 15))) // 초음파 센서 측정값 튀는걸 필터링
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
        *pd = preDistance;
      }
  }
  else
  preDistance = *pd;
}
