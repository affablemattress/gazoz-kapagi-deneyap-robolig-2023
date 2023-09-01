#include "songs.h"
#include "musicPlayer.h"
#include "vars.h"
#include "Deneyap_Servo.h"
#include "esp_now.h"
#include "WiFi.h"
#include <Arduino.h>

#define DEBUG
/*
#ifdef DEBUG
Serial.println("");
#endif
*/

struct ControllerData {
  volatile int16_t analogX;
  volatile int16_t analogY;
  //volatile uint8_t switchX;
  //volatile uint8_t switchY;
  volatile uint8_t leftButton;
  volatile uint8_t rightButton;
};
SemaphoreHandle_t controllerDataMutex = NULL;
ControllerData myLilControllerData = { 
  .analogX = 0,
  .analogY = 0,
  .leftButton = 1, 
  .rightButton = 0
 };

const uint8_t pmk[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
esp_now_peer_info_t controllerPeerInfo = {
  //TODO FIX IP
  .peer_addr = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  .lmk = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
  .channel = 3,
  .ifidx = WIFI_IF_STA,
  .encrypt = 1,
  .priv = NULL
};

class MotorPWM {
  uint8_t _channel;
  uint8_t _pinEn;
  uint8_t _pin1;
  uint8_t _pin2;
  uint8_t _lastDirection = 0; //0 reverse, 1 forward
public:
  MotorPWM() {
    
  }
  //TODO CHECK IF BIT IS GOOD IF 8 BIT REQUIRED DIVIDE DUTY BY 4
  MotorPWM(uint8_t chan, uint8_t pinEn, uint8_t pin1, uint8_t pin2)
    : _channel(chan), _pinEn(pinEn), _pin1(pin1), _pin2(pin2){
    ledcSetup(chan, 5000, 13);
    ledcAttachPin(pinEn, chan);
    write(0);
  }
  //Duty MUST be between [-1000 - 1000]
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
Servo servoGrip;
Servo servoTrig;

void espNowReceiveCb(const uint8_t* mac, const uint8_t* incomingData, int len);
//void gripPressedCb();

void setup() {
  controllerDataMutex = xSemaphoreCreateMutex();

//<------------------------------------------------------------------------------>
//<----------------------------------SETUP PINS---------------------------------->
//<------------------------------------------------------------------------------>
  pinMode(BUT1, INPUT_PULLUP);
  pinMode(BUT2, INPUT_PULLUP);
  pinMode(MY_LEDG, OUTPUT);
  pinMode(MY_LEDB, OUTPUT);
  pinMode(PWML, OUTPUT);
  pinMode(PWMR, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(BUZZ, OUTPUT);
  pinMode(SV_GRIP, OUTPUT);
  pinMode(SV_TRIG, OUTPUT);

  //attachInterrupt(digitalPinToInterrupt(BUT1), gripButtonInterrupt, FALLING);

//<------------------------------------------------------------------------------>
//<-------------------------------SETUP ESP_NOW---------------------------------->
//<------------------------------------------------------------------------------>
  WiFi.mode(WIFI_STA);
  while(esp_now_init() != 0) {
    #ifdef DEBUG
    Serial.println("ESP_NOW setup failed. Retrying in 500ms...");
    #endif
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  esp_now_set_pmk(pmk);
  esp_now_add_peer(&controllerPeerInfo);
  esp_now_register_recv_cb(espNowReceiveCb);
  #ifdef DEBUG
  Serial.println("ESP_NOW setup successful!");
  #endif

//<------------------------------------------------------------------------------>
//<--------------------------------SETUP SERVOS---------------------------------->
//<------------------------------------------------------------------------------>
  #ifdef DEBUG
  Serial.println("Starting up servos...");
  #endif
  servoGrip.attach(SV_GRIP, 0);
  servoTrig.attach(SV_TRIG, 1);
  servoGrip.write(GRIP_RELAX_ANGLE);
  servoTrig.write(TRIG_DEF_ANGLE);

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

//<------------------------------------------------------------------------------>
//<------------------------------WAIT FOR GRIP----------------------------------->
//<------------------------------------------------------------------------------>
  #ifdef DEBUG
  Serial.println("Waiting for grip button press...");
  #endif
  while(!myLilControllerData.leftButton) {}
  servoGrip.write(GRIP_HOLD_ANGLE);
  digitalWrite(MY_LEDB, 1);

  #ifdef DEBUG
  Serial.println("Grip engaged, ready to move!");
  #endif
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  xTaskCreatePinnedToCore((TaskFunction_t)playerTask, "Player", 1024, (void*)&master, 0, &playerTaskHandle, APP_CPU_NUM);
}

void loop() {
  static ControllerData mySatanicControllerData = { 0 };
  xSemaphoreTake(controllerDataMutex, 10);
  memcpy(&mySatanicControllerData, &myLilControllerData, sizeof(ControllerData));
  xSemaphoreGive(controllerDataMutex);

  #ifdef DEBUG
  static char printBuffer[100];
  sprintf(printBuffer, "Analog X: %d - Analog Y: %d - Left Button: %u - Right Button: %u\n", mySatanicControllerData.analogX, mySatanicControllerData.analogY, mySatanicControllerData.leftButton, mySatanicControllerData.rightButton);
  Serial.print(printBuffer);
  #endif
  //TODO FIGURE THIS OUT
  leftMotor.write(mySatanicControllerData.analogY + mySatanicControllerData.analogX);
  rightMotor.write(mySatanicControllerData.analogY - mySatanicControllerData.analogX);

  if(mySatanicControllerData.leftButton) {
    servoGrip.write(GRIP_RELAX_ANGLE);
    digitalWrite(MY_LEDB, 1);
  } 
  else {
    servoGrip.write(GRIP_HOLD_ANGLE);
  }

  if(mySatanicControllerData.rightButton) {
    servoTrig.write(TRIG_PRESSED_ANGLE);
  } 
  else {
    servoTrig.write(TRIG_DEF_ANGLE);
  }
}

void espNowReceiveCb(const uint8_t* mac, const uint8_t* incomingData, int len) {
  //TODO maybe critical setion..?
  static uint8_t ledCurrent = 0;
  if (!strncmp((char*)mac, (char*)controllerPeerInfo.peer_addr, 6)) {
    ledCurrent ^= 1;
    digitalWrite(MY_LEDG, ledCurrent);
    xSemaphoreTakeFromISR(controllerDataMutex, NULL);
    memcpy(&myLilControllerData, incomingData, sizeof(ControllerData));
    xSemaphoreGiveFromISR(controllerDataMutex, NULL);
  }
  else {
    #ifdef DEBUG
    ESP_LOGW(logTAG, "!IMPOSTER DETECTED! VUR KAHPEYE!");
    #endif 
  }
}

/*void gripButtonInterrupt() {
  volatile static uint32_t isr_db_counter = 0;
  if (millis() - isr_db_counter > DEBOUNCE_CONSTANT) {

    isr_db_counter = millis();
  }
}*/
