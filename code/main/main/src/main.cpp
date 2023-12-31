#include "songs.h"
#include "musicPlayer.h"
#include "vars.h"
#include "Deneyap_Servo.h"
#include "esp_now.h"
#include "WiFi.h"
#include <Arduino.h>

#define DEBUGa

struct ControllerData {
  volatile int16_t analogX;
  volatile int16_t analogY;
  volatile uint8_t baseServoPosition;
  volatile uint8_t hitServoPosition;
};
ControllerData myControllerData = {
  .analogX = 0,
  .analogY = 0,
  .baseServoPosition = 150,
  .hitServoPosition = HIT_SERVO_LOW
};

esp_now_peer_info_t controllerPeerInfo = {
  .peer_addr = { 0x7C, 0xDF, 0xA1, 0x93, 0x32, 0x46 },
  .lmk = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
  .channel = 0,
  .ifidx = WIFI_IF_STA,
  .encrypt = 0,
  .priv = NULL
};

class MotorPWM {
  uint8_t _channel;
  uint8_t _pinEn;
  uint8_t _pin1;
  uint8_t _pin2;
  uint8_t _lastDirection = 0; //0 reverse, 1 forward
public:
  MotorPWM() {}
  MotorPWM(uint8_t chan, uint8_t pinEn, uint8_t pin1, uint8_t pin2)
    : _channel(chan), _pinEn(pinEn), _pin1(pin1), _pin2(pin2){
    ledcSetup(chan, 5000, 13);
    ledcAttachPin(pinEn, chan);
    write(0);
  }
  inline void write(int16_t speed) {
    static uint8_t direction = 0;
    if(speed < 0) {
      direction = 0;
      speed *= -1;
    }
    else {
      direction = 1;
    }
    if(speed > 1000) {
      speed = 1000;
    }

    uint32_t dutyValue = (8191 / 1000) * speed;
    if(speed = 1000) {
      dutyValue = 8191;
    }

    ledcWrite(_channel, dutyValue);
    if(direction != _lastDirection) {
      _lastDirection = direction;
      digitalWrite(_pin1, direction);
      digitalWrite(_pin2, !direction);
    }
  }
};

MotorPWM leftMotor;
MotorPWM rightMotor;
Servo servoBase;
Servo servoHit;

void espNowReceiveCb(const uint8_t* mac, const uint8_t* incomingData, int len);

void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println(WiFi.macAddress());
  #endif

//<------------------------------------------------------------------------------>
//<----------------------------------SETUP PINS---------------------------------->
//<------------------------------------------------------------------------------>
  pinMode(BUT1, INPUT_PULLUP);
  pinMode(BUT2, INPUT_PULLUP);
  pinMode(MY_LEDR, OUTPUT);
  pinMode(MY_LEDG, OUTPUT);
  pinMode(PWML, OUTPUT);
  pinMode(PWMR, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(BUZZ, OUTPUT);
  pinMode(SV_BASE, OUTPUT);
  pinMode(SV_HIT, OUTPUT);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

//<------------------------------------------------------------------------------>
//<-------------------------------SETUP ESP_NOW---------------------------------->
//<------------------------------------------------------------------------------>
  WiFi.mode(WIFI_STA);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  while(esp_now_init() != 0) {
    #ifdef DEBUG
    Serial.println("ESP_NOW setup failed. Retrying in 500ms...");
    #endif
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  esp_now_add_peer(&controllerPeerInfo);
  esp_now_register_recv_cb(espNowReceiveCb);
  #ifdef DEBUG
  Serial.println("ESP_NOW setup successful!");
  #endif
  vTaskDelay(1000 / portTICK_PERIOD_MS);

//<------------------------------------------------------------------------------>
//<--------------------------------SETUP SERVOS---------------------------------->
//<------------------------------------------------------------------------------>
  #ifdef DEBUG
  Serial.println("Starting up servos...");
  #endif
  servoBase.attach(SV_BASE, 0);
  servoHit.attach(SV_HIT, 1);
  servoBase.write(150);
  servoHit.write(HIT_SERVO_LOW);
  vTaskDelay(500 / portTICK_PERIOD_MS);

  leftMotor = MotorPWM(2, PWML, IN1, IN2);
  rightMotor = MotorPWM(3, PWMR, IN3, IN4);
  leftMotor.write(1000);
  rightMotor.write(-1000);
  vTaskDelay(500 / portTICK_PERIOD_MS);
  leftMotor.write(-1000);
  rightMotor.write(1000);
  vTaskDelay(500 / portTICK_PERIOD_MS);
  leftMotor.write(0);
  rightMotor.write(0);

  vTaskDelay(1000 / portTICK_PERIOD_MS);
  //xTaskCreatePinnedToCore((TaskFunction_t)playerTask, "Player", 1024, (void*)&master, 0, &playerTaskHandle, APP_CPU_NUM);
}

void loop() {
  static ControllerData mySatanicControllerData = { 0 };
  static int16_t speedLeft = 0;
  static int16_t speedRight = 0;

  memcpy(&mySatanicControllerData, &myControllerData, sizeof(ControllerData));

  speedLeft = mySatanicControllerData.analogY + mySatanicControllerData.analogX;
  speedRight = mySatanicControllerData.analogY - mySatanicControllerData.analogX;

  leftMotor.write(speedLeft);
  rightMotor.write(speedRight);

  servoBase.write(mySatanicControllerData.baseServoPosition);
  servoBase.write(mySatanicControllerData.hitServoPosition);
  
  #ifdef DEBUG
  static char printBuffer[150];
  sprintf(printBuffer, "Speed L: %d - Speed R: %d\nBase: %u - Hit: %u\n", speedLeft, speedRight, mySatanicControllerData.baseServoPosition, mySatanicControllerData.hitServoPosition);
  Serial.print(printBuffer);
  #endif
}

void espNowReceiveCb(const uint8_t* mac, const uint8_t* incomingData, int len) {
  static uint8_t ledCurrent = 0;
  if (!strncmp((char*)mac, (char*)controllerPeerInfo.peer_addr, 6)) {
    ledCurrent ^= 1;
    digitalWrite(MY_LEDG, ledCurrent);
    memcpy(&myControllerData, incomingData, sizeof(ControllerData));
  }
}
