#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

TimerHandle_t one_shot_timer = NULL;

void blinkTask(void* pvParameters);
void myTimerCallBack(TimerHandle_t xTimer);

void setup() {
  Serial.begin(9600);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
 
  one_shot_timer = xTimerCreate(
    "One Shot Timer",
    1 / portTICK_PERIOD_MS,
    pdFALSE,
    (void *)0,
    myTimerCallBack
  );

  xTaskCreatePinnedToCore(
    blinkTask,
    "Blink Task",
    1024,
    NULL,
    1,
    NULL,
    app_cpu
  );

  pinMode(LED_BUILTIN, OUTPUT);

  vTaskDelete(NULL);

}

void blinkTask(void *pvParameters){
  char c;
  while(1){
    if(Serial.available() > 0){
      c = Serial.read();
      Serial.print(c);
      
      digitalWrite(LED_BUILTIN, HIGH);

      xTimerStart(one_shot_timer, portMAX_DELAY);
    }
  }
}

void myTimerCallBack(TimerHandle_t xTimer){
  vTaskDelay(3000 / portTICK_PERIOD_MS);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

}
