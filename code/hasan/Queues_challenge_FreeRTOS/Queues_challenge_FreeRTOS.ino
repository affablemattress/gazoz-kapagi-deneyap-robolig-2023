// Use only 1 core for demo purpose
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

#include <stdlib.h>

//  FUNCTIONS
void taskA(void *pvParameters);
void taskB(void *pvParameters);

//  GLOBALS
static QueueHandle_t queue1, queue2;
static TaskHandle_t a_handler = NULL, b_handler = NULL;

//  SETTINGS
static const uint8_t q_len = 5;

void setup() {
  Serial.begin(9600);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  queue1 = xQueueCreate(q_len, sizeof(uint16_t));
  queue2 = xQueueCreate(q_len, sizeof(uint16_t));

  xTaskCreatePinnedToCore(
    taskA,
    "Task A",
    2048,
    NULL,
    1,
    &a_handler,
    app_cpu
  );

  xTaskCreatePinnedToCore(
    taskB,
    "Task B",
    1024,
    NULL,
    1,
    &b_handler,
    app_cpu
  );

  vTaskDelete(NULL);
}

void taskA(void *pvParameters){
  uint8_t buf_len = 255;
  char c;
  char msg[buf_len];
  uint8_t idx;

  uint16_t received_item;
  while(1){
    if(Serial.available() > 0){
      c = Serial.read();
      if(c == '\n'){

        Serial.println(msg);

        if(msg[0] == 'd'&& msg[1] == 'e' && msg[2] == 'l' && msg[3] == 'a' && msg[4] == 'y'){
          char buf[buf_len];
          for(int i = 6 ; i < buf_len ; i ++){
            buf[i-6] = msg[i];
          }
          uint16_t delay_time = atoi(buf);
          Serial.println(delay_time);
          if(xQueueSend(queue1, (void*)& delay_time, 10) != pdTRUE){
            Serial.println("Queue1 is full!");
          }
        }

        memset(msg, 0, buf_len);
        idx = 0;
      }
      else{
          if(idx < buf_len - 1){  
          msg[idx] = c;
          idx++;
        }
      }
    }

    if(xQueueReceive(queue2, (void *)&received_item, 0) == pdTRUE){
      Serial.print("BLINKED : ");
      Serial.println(received_item);
    }
  }
}

void taskB(void *pvParameters){
  pinMode(2, OUTPUT);
  uint16_t delay_time = 500;
  uint16_t true_false;
  uint16_t blink_counter;
  while(1){
    if(xQueueReceive(queue1, (void *)&delay_time, 0) == pdTRUE){
      Serial.println("*************");
    }
    if(blink_counter == 99){
      true_false = 1;
      if(xQueueSend(queue2, (void*)& true_false, 10) != pdTRUE){
        Serial.println("Queue2 is full!");
      }
      true_false = 0;
      blink_counter = 0;
    }
    digitalWrite(2, HIGH);
    vTaskDelay(delay_time / portTICK_PERIOD_MS);
    digitalWrite(2, LOW);
    vTaskDelay(delay_time / portTICK_PERIOD_MS);
    blink_counter ++;
  }
}

void loop() {

}
