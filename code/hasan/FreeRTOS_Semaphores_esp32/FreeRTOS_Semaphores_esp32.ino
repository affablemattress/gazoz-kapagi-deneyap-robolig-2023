#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

SemaphoreHandle_t mySemaphore, mutex;

typedef struct message{
  char body[20];
  uint8_t len;
}message;

void printer_task(void *pvParameters);

void setup() {
  Serial.begin(9600);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  mySemaphore = xSemaphoreCreateCounting(5, 0);
  mutex = xSemaphoreCreateMutex();

  message msg;
  strcpy(msg.body, "Selam Dünya!");
  msg.len = strlen("Selam Dünya!");
  
  char task_name[10];

  for(int i = 0 ; i < 5 ; i++){
    sprintf(task_name, "Task %i", i );
    xTaskCreatePinnedToCore(
      printer_task,
      task_name,
      1024,
      (void *)& msg,
      1,
      NULL,
      app_cpu
    );
    memset(task_name, 0, 10);
  }
  for(int i = 0 ; i < 5 ; i++){
    xSemaphoreTake(mySemaphore, portMAX_DELAY);
  }
  Serial.println();  
  Serial.println("All tasks are created!");
}

void printer_task(void *pvParameters){
  message msg = *(message*) pvParameters;

  xSemaphoreTake(mutex, portMAX_DELAY);
  Serial.println();
  Serial.print("Message body: ");
  Serial.print(msg.body);
  Serial.print(" | length: ");
  Serial.println(msg.len);
  
  xSemaphoreGive(mutex);

  xSemaphoreGive(mySemaphore);

  vTaskDelay(1000 / portTICK_PERIOD_MS);
  vTaskDelete(NULL);
}

void loop() {
  // put your main code here, to run repeatedly:

}
