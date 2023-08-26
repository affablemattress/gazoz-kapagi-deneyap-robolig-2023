// Use only 1 core for demo purpose
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

void testTask(void *pvParameters);

static TaskHandle_t testTask_handler;

void setup() {
  Serial.begin(38400);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  
  xTaskCreatePinnedToCore(
    testTask,
    "Test Task",
    1500,
    NULL,
    1,
    &testTask_handler,
    app_cpu
  );

  vTaskDelete(NULL); //delete setup and loop tasks
}

void testTask(void *pvParameters){
  while(1){
    int a = 0;
    int b[100];

    for(int i = 0 ; i < 100 ; i++){
      b[i] = a + 1; 
    }
    Serial.println(b[0]);

    // print out number of free heap memory bytes before malloc
    Serial.print("Heap before malloc bytes: ");
    Serial.println(xPortGetFreeHeapSize());

    int* ptr = (int*)pvPortMalloc(1024 * sizeof(int));

    if(ptr == NULL){
      Serial.println("Not enough heap to allocate"); // it only prevents the error but doesn't optimize the program.
    }
    else{
      // overflow
      for(int a = 0 ; a < 1024 ; a ++){
        ptr[a] = 3;
      }
    }

    // print out number of free heap memory bytes after malloc
    Serial.print("Heap before malloc bytes: ");
    Serial.println(xPortGetFreeHeapSize());

    vPortFree(ptr); // true solution
    // now our program is thread safe
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
