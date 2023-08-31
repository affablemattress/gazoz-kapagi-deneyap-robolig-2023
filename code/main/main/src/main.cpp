#include "esp_now.h"
#include "WiFi.h"
#include <Arduino.h>

#define DEBUG
/*
#ifdef DEBUG
Serial.println("");
#endif
*/

namespace{
  #define BUT1 D0
  #define BUT2 D1

  #define LEDG A4
  #define LEDB A5

  #define PWML D4
  #define PWMR SCK

  #define IN1 MISO
  #define IN2 MOSI
  #define IN3 D8
  #define IN4 D9

  #define BUZZ D15

  #define SV_GRIP DAC1
  #define SV_TRIG DAC2
}

struct controllerData {
  volatile uint16_t analogX;
  volatile uint16_t analogY;
  volatile uint8_t switchX;
  volatile uint8_t switchY;
  volatile uint8_t leftButton;
  volatile uint8_t centerButton;
  volatile uint8_t rightButton;
};
SemaphoreHandle_t controllerDataMutex = NULL;
controllerData myLilControllerData;

void espNowReceiveCb(const uint8_t* mac, const uint8_t* incomingData, int len);

const uint8_t pmk[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
esp_now_peer_info_t controllerPeerInfo = {
    .peer_addr = { 0x3C, 0xE9, 0x0E, 0x86, 0x0C, 0x84 },
    .lmk = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
    .channel = 0,
    .ifidx = WIFI_IF_STA,
    .encrypt = 1,
    .priv = NULL
};

void setup() {
  controllerDataMutex = xSemaphoreCreateMutex();

//<------------------------------------------------------------------------------>
//<----------------------------------SETUP PINS---------------------------------->
//<------------------------------------------------------------------------------>
  pinMode(BUT1, INPUT_PULLUP);
  pinMode(BUT2, INPUT_PULLUP);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(PWML, OUTPUT);
  pinMode(PWMR, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(BUZZ, OUTPUT);
  pinMode(SV_GRIP, OUTPUT);
  pinMode(SV_TRIG, OUTPUT);

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
  
}

void loop() {
  
}

void espNowReceiveCb(const uint8_t* mac, const uint8_t* incomingData, int len) {
  //maybe critical section?...
  if (!strncmp((char*)mac, (char*)controllerPeerInfo.peer_addr, 6)) {
    xSemaphoreTakeFromISR(controllerDataMutex, NULL);
    memcpy(&controller_data, incomingData, sizeof(controllerData));
    xSemaphoreGiveFromISR(controllerDataMutex, NULL);
  }
  else {
    #ifdef DEBUG
    ESP_LOGW(logTAG, "!IMPOSTER DETECTED! VUR KAHPEYE!");
    #endif 
  }
}
