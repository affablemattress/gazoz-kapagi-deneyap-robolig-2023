#include "esp_now.h"
#include "WiFi.h"
#include <Arduino.h>

#define POLL_PERIOD 50

#define DEBUG
static const char* logTAG = "MainLog";

SemaphoreHandle_t controllerDataMutex = NULL;
struct controllerData {
    uint16_t analogX;
    uint16_t analogY;
    int8_t speed;  //positive -> forward
    int8_t rotational;  //positive -> right 
    uint8_t modSelect;
};

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

//    object for data receiving
controllerData controller_data;

void setup() {
  #ifdef DEBUG
    esp_log_level_set(logTAG, ESP_LOG_ERROR);
  #endif

  controllerDataMutex = xSemaphoreCreateMutex();x
  WiFi.mode(WIFI_STA);
  while(esp_now_init() != 0) {
    #ifdef DEBUG
    ESP_LOGE(logTAG, "ESP_NOW init failed!");
    #endif
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  #ifdef DEBUG
  ESP_LOGE(logTAG, "ESP_NOW init failed!");
  #endif
  
  esp_now_set_pmk(pmk);
  esp_now_add_peer(&controllerPeerInfo);
  esp_now_recv_cb_t(espNowReceiveCb);
}

void loop() {
  
}

void espNowReceiveCb(const uint8_t* mac, const uint8_t* incomingData, int len) {
  //maybe critical section?...
  if (!strncmp((char*)mac, (char*)controllerPeerInfo.peer_addr, 6)) {
    xSemaphoreTakeFromISR(controllerDataMutex, POLL_PERIOD);
    memcpy(&controller_data, incomingData, sizeof(controllerData));
    xSemaphoreGiveFromISR(controllerDataMutex);
  }
  else {
    #ifdef DEBUG
    ESP_LOGW(logTAG, "!IMPOSTER DETECTED! VUR KAHPEYE!");
    #endif 
  }
}
