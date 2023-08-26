// Use only 1 core for demo purpose
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

static TaskHandle_t sender_handler = NULL, receiver_handler = NULL;

void sender_task(void *pvParameters);
void receiver_task(void *pvParameters);

void setup() {
  Serial.begin(38400);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  xTaskCreatePinnedToCore(
    sender_task,
    "Sender Task",
    1024,
    NULL,
    1,
    &sender_handler,
    app_cpu
  );

  xTaskCreatePinnedToCore(
    receiver_task,
    "Receiver Task",
    1024,
    NULL,
    1,
    &receiver_handler,
    app_cpu
  );

  vTaskDelete(NULL);
}

// settings
static const uint8_t buf_len = 255;

// globals
static char *msg = NULL;
static volatile uint8_t msg_flag = 0;

void sender_task(void *pvParameters){
  char c;
  uint8_t cidx;
  char buf[buf_len];
  while(1){
    if(Serial.available() > 0){
      c = Serial.read();

      if(c == '\n'){
        buf[cidx - 1] = '\0'; // making it null terminated
        
        if(msg_flag == 0){
          msg = (char*)pvPortMalloc(cidx * sizeof(char));
          memcpy(msg, buf, cidx);
          msg_flag = 1;
        }
        
        memset(buf, 0, buf_len);
        cidx = 0;
      }

      else{
        if(cidx < buf_len - 1){
          buf[cidx] = c;
          cidx ++;
        }
      }
    }
  }
}

void receiver_task(void *pvParameters){
  while(1){
    if(msg_flag == 1){

    Serial.println(msg);

    vPortFree(msg);
    msg = NULL;
    msg_flag = 0; 
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
