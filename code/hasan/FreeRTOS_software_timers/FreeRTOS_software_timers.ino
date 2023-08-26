#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

TimerHandle_t one_shot_handler = NULL, auto_reload_handler = NULL;

void myTimerCallBack(TimerHandle_t xTimer){
  if((uint32_t)pvTimerGetTimerID(xTimer) == 0){
    Serial.println("one_shot_timer expired!");
  }
  else if((uint32_t)pvTimerGetTimerID(xTimer) == 1){
    Serial.println("auto_reload_timer expired!");
  }
}

void setup() {
  Serial.begin(9600);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  one_shot_handler = xTimerCreate(
    "One Shot Timer",
    2000 / portTICK_PERIOD_MS,
    pdFALSE,    // one_shot
    (void *)0,  // id
    myTimerCallBack
  );

  auto_reload_handler = xTimerCreate(
    "Auto Reload Timer",
    1000 / portTICK_PERIOD_MS,
    pdTRUE,    // one_shot
    (void *)1,  // id
    myTimerCallBack
  );

  if(one_shot_handler == NULL || auto_reload_handler == NULL){
    Serial.println("Timers cannot be created");
  }

  xTimerStart(one_shot_handler, portMAX_DELAY);
  xTimerStart(auto_reload_handler, portMAX_DELAY);

  vTaskDelete(NULL);
}

void loop() {
  // put your main code here, to run repeatedly:

}
