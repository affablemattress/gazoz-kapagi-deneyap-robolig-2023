#include "vars.h"
#include "esp_now.h"
#include "WiFi.h"
#include <Arduino.h>

#define DEBUG
/*
#ifdef DEBUG
Serial.println("");
#endif
*/

SemaphoreHandle_t controllerDataMutex = NULL;
struct ControllerData {
  volatile uint16_t analogX;
  volatile uint16_t analogY;
  //volatile uint8_t switchX;
  //volatile uint8_t switchY;
  volatile uint8_t leftButton;
  volatile uint8_t rightButton;
};
ControllerData myControllerData = {
  .analogX = 2048,
  .analogY = 2048,
  //.switchX = 0,
  //.switchY = 0,
  .leftButton = 1,
  .rightButton = 0,
};
uint16_t bufX[4];
uint16_t bufY[4];

const uint8_t pmk[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
esp_now_peer_info_t mainPeerInfo = {
  //TODO FIX IP
  .peer_addr = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  .lmk = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
  .channel = 3,
  .ifidx = WIFI_IF_STA,
  .encrypt = 1,
  .priv = NULL
};

void interruptLB();
void interruptCB();
void interruptRB();
//void interruptSW_X();
//void interruptSW_Y();

void setup() {
  controllerDataMutex = xSemaphoreCreateMutex();
  memset(bufX, 2048, 4);
  memset(bufY, 2048, 4);

//<------------------------------------------------------------------------------>
//<----------------------------------SETUP PINS---------------------------------->
//<------------------------------------------------------------------------------>
  pinMode(LB, INPUT_PULLUP);
  pinMode(CB, INPUT_PULLUP);
  pinMode(RB, INPUT_PULLUP);
  //pinMode(SW_X, INPUT_PULLUP);
  //pinMode(SW_Y, INPUT_PULLUP);

  pinMode(MY_LEDB, OUTPUT);
  digitalWrite(MY_LEDB, 1);
  pinMode(MY_LEDG, OUTPUT);

  pinMode(JOYX, INPUT);
  pinMode(JOYY, INPUT);

  pinMode(BUZZ, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(LB), interruptLB, FALLING);
  attachInterrupt(digitalPinToInterrupt(CB), interruptCB, FALLING);
  attachInterrupt(digitalPinToInterrupt(RB), interruptRB, FALLING);
  //attachInterrupt(digitalPinToInterrupt(SW_X), interruptSW_X, FALLING);
  //attachInterrupt(digitalPinToInterrupt(SW_Y), interruptSW_Y, FALLING);

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
  esp_now_add_peer(&mainPeerInfo);
  #ifdef DEBUG
  Serial.println("ESP_NOW setup successful!");
  #endif
}

void loop() {
  static uint16_t rollingAvgX = 2048;
  static uint16_t rollingAvgY = 2048;

  memmove(bufX, bufX + 1, sizeof(uint16_t) * 3);
  bufX[3] = analogRead(JOYX);
  memmove(bufY, bufY + 1, sizeof(uint16_t) * 3);
  bufY[3] = analogRead(JOYY);
  rollingAvgX = (bufX[0] + bufX[1] + bufX[2] + bufX[3]) / 4;
  rollingAvgY = (bufY[0] + bufY[1] + bufY[2] + bufY[3]) / 4;

  xSemaphoreTake(controllerDataMutex, 10);
  myControllerData.analogX = rollingAvgX * 1000 / 4096;
  myControllerData.analogY = rollingAvgY * 1000 / 4096;;
  esp_now_send(mainPeerInfo.peer_addr, (uint8_t *) &myControllerData, sizeof(myControllerData));
  xSemaphoreGive(controllerDataMutex);
  vTaskDelay(30);
}

// TODO BUTTON LOGIC
void interruptLB() {
  volatile static uint32_t isr_db_counter = 0;
  if (millis() - isr_db_counter > DEBOUNCE_CONSTANT) {
    myControllerData.leftButton ^= 0b00000001;
    digitalWrite(MY_LEDB, myControllerData.leftButton);
    isr_db_counter = millis();
  }
}

void interruptCB() {
  volatile static uint32_t isr_db_counter = 0;
  if (millis() - isr_db_counter > DEBOUNCE_CONSTANT) {
    myControllerData.leftButton = 1;
    digitalWrite(MY_LEDB, 1);
    myControllerData.rightButton = 0;
    digitalWrite(MY_LEDG, 0);
    isr_db_counter = millis();
  }
}

void interruptRB() {
  volatile static uint32_t isr_db_counter = 0;
  if (millis() - isr_db_counter > DEBOUNCE_CONSTANT) {
    myControllerData.rightButton ^= 0b00000001;
    digitalWrite(MY_LEDG, myControllerData.rightButton);
    isr_db_counter = millis();
  }
}

/*void interruptSW_X() {

}

void interruptSW_Y() {

}*/
