// Use only 1 core for demo purpose
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

#include <stdlib.h>

static const uint8_t led_pin = LED_BUILTIN;

static TaskHandle_t Serial_task_handler = NULL, LED_blink_handler = NULL;
static SemaphoreHandle_t xMutex = NULL;

void Serial_task(void *pvParameters);
void LED_task(void *pvParameters);

void setup() {
  
  // Serial connection
  Serial.begin(9600);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Mutex declaration
  xMutex = xSemaphoreCreateBinary();
  while(xMutex == NULL){
    Serial.println("Semaphore is null!!");
  }

    pinMode(led_pin, OUTPUT);

  // Tasks
  xTaskCreatePinnedToCore(
    Serial_task,
    "Serial Task",
    1024,
    NULL,
    1,
    &Serial_task_handler,
    app_cpu
  );
  xTaskCreatePinnedToCore(
    LED_task,
    "LED Task",
    1024,
    NULL,
    1,
    &LED_blink_handler,
    app_cpu
  );
}

uint16_t delay_time = 500;

void Serial_task(void *pvParameters){
  uint8_t buf_len = 20;
  uint8_t c;
  uint8_t cIndex;
  char buf[buf_len];
  memset(buf, 0, buf_len);
  while(1){
    if(Serial.available() > 0){
      c = Serial.read();
      
      if(c == '\n'){
        xSemaphoreTake(xMutex, 100);
        delay_time = atoi(buf);
        Serial.println(delay_time);
        memset(buf, 0, buf_len);
        cIndex = 0;
      }
      else{
        if (cIndex < buf_len - 1) {
          buf[cIndex] = c;
          cIndex ++;
        }
      }    
    }  
  }
}

void LED_task(void *pvParameters){
  uint16_t time_variable;
  while(1){
    xSemaphoreTake(xMutex, 100);
    time_variable = delay_time;
    xSemaphoreGive(xMutex);
    digitalWrite(led_pin, HIGH);
    vTaskDelay(delay_time / portTICK_PERIOD_MS);
    digitalWrite(led_pin, LOW);
    vTaskDelay(delay_time / portTICK_PERIOD_MS);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
