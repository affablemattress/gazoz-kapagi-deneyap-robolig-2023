// Use only 1 core for demo purpose
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

static TaskHandle_t task1_handler, task2_handler;

void task1(void *pvParameters){
  char msg[] = "Hello FreeRTOS World!!";
  uint8_t msg_len = strlen(msg);
  while(1){
    for(int i = 0 ; i< msg_len ; i++){
      Serial.print(msg[i]);
    }
    Serial.println();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void task2(void *pvParameters){
  while(1){
    Serial.print("*");
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {

  Serial.begin(300);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  Serial.println(xPortGetCoreID());
  Serial.println(uxTaskPriorityGet(NULL));

  xTaskCreatePinnedToCore(
    task1,
    "task 1",
    1024,
    NULL,
    1,
    &task1_handler,
    app_cpu
  );

  xTaskCreatePinnedToCore(
    task2,
    "task 2",
    1024,
    NULL,
    2,
    &task2_handler,
    app_cpu
  );
  
}

void loop() {
  for(int i = 0 ; i < 3 ; i++){
    vTaskSuspend(task2_handler);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    vTaskResume(task2_handler);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }

  if(task1_handler != NULL){
    vTaskDelete(task1_handler);
    task1_handler = NULL;  
  }
}