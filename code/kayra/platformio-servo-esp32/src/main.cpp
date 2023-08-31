#include <Arduino.h>
#include "driver/ledc.h"

#define TIMER_DUTY_RESOLUTION 15
#define TIMER_DUTY_MAX 32768 //should be set accourding to resolution 2**res

class Servo {
  private:
  ledc_channel_t _channel;
  gpio_num_t _pin;
  ledc_channel_config_t _channelConfig;
  ledc_timer_config_t _timerConfig;

  public:
  float pos;
  inline void setPos(float newPos) {
    ledc_timer_config(&_timerConfig);
    ledc_channel_config(&_channelConfig);

    ledc_set_duty_and_update((ledc_mode_t)(_channel / 8), _channel,
                             (uint32_t)((TIMER_DUTY_MAX / 20) + pos * (TIMER_DUTY_MAX / 20)), 0); 
  }

  Servo(Servo&&) = delete;
  Servo(const Servo&) = delete;
  Servo(ledc_channel_t channelLEDC, gpio_num_t numGPIO, float initPos)
  : _channel(channelLEDC), _pin(numGPIO), pos(initPos) {
    _timerConfig.speed_mode = (ledc_mode_t)(_channel / 8);
    _timerConfig.duty_resolution = (ledc_timer_bit_t)TIMER_DUTY_RESOLUTION;
    _timerConfig.timer_num = (ledc_timer_t)((_channel/2)%4);
    _timerConfig.freq_hz = 50;
    _timerConfig.clk_cfg = LEDC_USE_APB_CLK;

    _channelConfig.gpio_num = _pin;
    _channelConfig.speed_mode = LEDC_LOW_SPEED_MODE;
    _channelConfig.channel = _channel;
    _channelConfig.intr_type = LEDC_INTR_DISABLE;
    _channelConfig.timer_sel = (ledc_timer_t)((_channel/2)%4);
    _channelConfig.duty = (uint32_t)((TIMER_DUTY_MAX / 20) + pos * (TIMER_DUTY_MAX / 20));
    _channelConfig.hpoint = 0;

    this->setPos(0.5f);
  }
};

struct ServoData {
  SemaphoreHandle_t mutex;
  float trackedPos;
  Servo* servo;
};
void servoTask(void*);

void positionRandomizerTask(void*);

void setup() {
  Servo myServo(LEDC_CHANNEL_0, GPIO_NUM_14, 0.5f);
  ServoData myServoTaskData = {
    .mutex = xSemaphoreCreateMutex(),
    .trackedPos = 0.5f, 
    .servo = &myServo
  };
  vTaskDelay(1);

  TaskHandle_t servoTaskHandle = NULL;
  xTaskCreatePinnedToCore((TaskFunction_t)servoTask, "Servo Task", 1000, &myServoTaskData, 2, &servoTaskHandle, APP_CPU_NUM);

  TaskHandle_t positionRandomizerTaskHandle = NULL;
  xTaskCreatePinnedToCore((TaskFunction_t)positionRandomizerTask, "Position Randomizer Task", 1000, &myServoTaskData.trackedPos, 2, &positionRandomizerTaskHandle, APP_CPU_NUM);

  while(1) {}
}

void servoTask(void* servoDataV) {
  ServoData* servoData = (ServoData*)servoDataV;
  float currentPos = servoData->trackedPos;

  while(1) {
    float newPos = servoData->trackedPos;
    xSemaphoreTake(servoData->mutex, portMAX_DELAY);
    if(newPos != servoData->servo->pos) {
      servoData->servo->setPos(newPos);
      currentPos = newPos;
    }
    xSemaphoreGive(servoData->mutex);
    vTaskDelay(50);
  }
}

void positionRandomizerTask(void* servoDataV) {
  ServoData* servoData = (ServoData*)servoDataV;
  uint32_t randomGen = 2;
  while(1) {
    xSemaphoreTake(servoData->mutex, 10);
    randomGen *= randomGen + 1;
    servoData->trackedPos = 0.1f * (randomGen % 10);
    xSemaphoreGive(servoData->mutex);
    vTaskDelay(2000);
  }
}

void loop() {
  vTaskDelete(NULL);
}
