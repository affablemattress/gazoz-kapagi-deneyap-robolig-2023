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

struct ControllerData {
  volatile uint16_t analogX;
  volatile uint16_t analogY;
  volatile uint8_t switchX;
  volatile uint8_t switchY;
  volatile uint8_t leftButton;
  volatile uint8_t centerButton;
  volatile uint8_t rightButton;
};
ControllerData myControllerData = { 0 };

const uint8_t pmk[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
esp_now_peer_info_t mainPeerInfo = {
  //TODO FIX IP
  .peer_addr = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  .lmk = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
  .channel = 0,
  .ifidx = WIFI_IF_STA,
  .encrypt = 1,
  .priv = NULL
};

void interruptLB();
void interruptCB();
void interruptRB();
void interruptSW_X();
void interruptSW_Y();

void setup() {

//<------------------------------------------------------------------------------>
//<----------------------------------SETUP PINS---------------------------------->
//<------------------------------------------------------------------------------>
  pinMode(LB, INPUT_PULLUP);
  pinMode(CB, INPUT_PULLUP);
  pinMode(RB, INPUT_PULLUP);
  pinMode(SW_X, INPUT_PULLUP);
  pinMode(SW_Y, INPUT_PULLUP);

  pinMode(LEDB, OUTPUT);
  pinMode(LEDG, OUTPUT);

  pinMode(JOYX, INPUT);
  pinMode(JOYY, INPUT);

  pinMode(BUZZ, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(LB), interruptLB, FALLING);
  attachInterrupt(digitalPinToInterrupt(CB), interruptCB, FALLING);
  attachInterrupt(digitalPinToInterrupt(RB), interruptRB, FALLING);
  attachInterrupt(digitalPinToInterrupt(SW_X), interruptSW_X, FALLING);
  attachInterrupt(digitalPinToInterrupt(SW_Y), interruptSW_Y, FALLING);

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
  static uint16_t bufX1[4];
  static uint16_t bufX2[3];
  static uint16_t bufY1[4];
  static uint16_t bufY2[3];
  
  bufX1[3] = analogRead(JOYX);                        //    X_axis joystick value
  memmove(&bufX2, &bufX1, sizeof(uint16_t) * 3);

  bufY1[3] = analogRead(JOYY);                        //    Y_axis joystick value
  memmove(&bufY2, &bufY1, sizeof(uint16_t) * 3);


  esp_now_send(mainPeerInfo.peer_addr, (uint8_t *) &myControllerData, sizeof(myControllerData));

}

// TODO BUTTON LOGIC
void gripPressedCb() {
  volatile static uint32_t isr_db_counter = 0;
  if (millis() - isr_db_counter > DEBOUNCE_CONSTANT) {

    isr_db_counter = millis();
  }
}

void gripPressedCb() {
  volatile static uint32_t isr_db_counter = 0;
  if (millis() - isr_db_counter > DEBOUNCE_CONSTANT) {

    isr_db_counter = millis();
  }
}

void gripPressedCb() {
  volatile static uint32_t isr_db_counter = 0;
  if (millis() - isr_db_counter > DEBOUNCE_CONSTANT) {

    isr_db_counter = millis();
  }
}
