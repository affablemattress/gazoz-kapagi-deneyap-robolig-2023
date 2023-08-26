// Use only 1 core for demo purpose
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

//  Globals
uint16_t my_integer;
SemaphoreHandle_t myMutex;

//  Settings

//  Functions
void task_a(void *pvParameters);

void setup() {
  Serial.begin(600);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  myMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
    task_a,
    "Task A",
    1024,
    NULL,
    1,
    NULL,
    app_cpu
  );

  xTaskCreatePinnedToCore(
    task_a,
    "Task B",
    1024,
    NULL,
    1,
    NULL,
    app_cpu
  );
  
  vTaskDelete(NULL);
}

void task_a(void *pvParameters){
  uint16_t local_var;
  pinMode(LED_BUILTIN, OUTPUT);
  while(1){
    if(xSemaphoreTake(myMutex, 0) == pdTRUE){
      local_var = my_integer;
      local_var ++;
      vTaskDelay(random(50, 500) / portTICK_PERIOD_MS);
      my_integer = local_var;

      xSemaphoreGive(myMutex);
      Serial.println(my_integer);
    }else{
      digitalWrite(LED_BUILTIN, HIGH);
      vTaskDelay(25 / portTICK_PERIOD_MS);
      digitalWrite(LED_BUILTIN, LOW);
      vTaskDelay(25 / portTICK_PERIOD_MS);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
