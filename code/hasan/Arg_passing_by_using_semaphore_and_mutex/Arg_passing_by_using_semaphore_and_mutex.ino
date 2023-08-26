#if CONGIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else 
static const BaseType_t app_cpu = 1;
#endif

void blinkLED(void *pvParameters);

//SemaphoreHandle_t myMutex;
SemaphoreHandle_t binary_semphr;

void setup() {
  Serial.begin(9600);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  //myMutex = xSemaphoreCreateMutex();
  binary_semphr = xSemaphoreCreateBinary();

  Serial.println("Please type delay time: ");
  while(Serial.available() <= 0);
  uint16_t delay_time = Serial.parseInt();
  Serial.print("Delay: ");
  Serial.println(delay_time);

  xTaskCreatePinnedToCore(
    blinkLED,
    "Blink LED",
    1024,
    (void *)&delay_time,
    1,
    NULL,
    app_cpu
  );

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  //  xSemaphoreTake(myMutex, portMAX_DELAY); Mutex is a bit hack for this purpose!!
  xSemaphoreTake(binary_semphr, portMAX_DELAY); // binary semaphr blocks the setup function until it receives signal from xSemaphoreGive() function.
  Serial.println("Done!");


}

void blinkLED(void *pvParameters){
  
  uint16_t delay_arg = *(uint16_t *)pvParameters;

  //xSemaphoreGive(myMutex);
  xSemaphoreGive(binary_semphr);
  

  Serial.print("Received: ");
  Serial.println(delay_arg);

  pinMode(LED_BUILTIN, OUTPUT);
  while(1){
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(delay_arg / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(delay_arg / portTICK_PERIOD_MS);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
