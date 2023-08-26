#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

void task_a(void *pvParameters);
void task_b(void *pvParameters);
void onTimer();

// settings
static const uint16_t timer_divider = 500; //  Clock ticks at 10 Hz!
static const uint16_t timer_max_count = 1000000;
static const TickType_t taskDelay = 2000 / portTICK_PERIOD_MS;

// pins
static const int ledPin = LED_BUILTIN;

// globals
static hw_timer_t *timer = NULL;
static volatile int isr_counter;
static SemaphoreHandle_t bin_sem;
static int16_t buf_isr[10];
static int16_t buf_task[10];

void setup() {
  Serial.begin(9600);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  bin_sem = xSemaphoreCreateBinary();

  xTaskCreatePinnedToCore(
    task_a,
    "Task A",
    1024,
    NULL,
    2,
    NULL,
    app_cpu
  );

  xTaskCreatePinnedToCore(
    task_b,
    "Task B",
    1024,
    NULL,
    1,
    NULL,
    app_cpu
  );

  // analog pin
  pinMode(39, INPUT);

  // Create and start timer (num, divider, edge)
  timer = timerBegin(0, timer_divider, true);
  // Provide ISR to timer (timer, function, edge)
  timerAttachInterrupt(timer, &onTimer, true);
  // at what count should ISR trigger (timer, count, autoreload)
  timerAlarmWrite(timer, timer_max_count, true);
  // allows ISR to trigger
  timerAlarmEnable(timer);

  vTaskDelete(NULL);
}

uint8_t counter;

void IRAM_ATTR onTimer(){
  BaseType_t task_woken = pdFALSE;
  buf_isr[counter] = analogRead(39);
  counter ++;
  if(counter == 10){
    memcpy(&buf_task, &buf_isr, sizeof(int16_t) * 10);
    memset(&buf_isr, 0, 10);
    xSemaphoreGiveFromISR(bin_sem, &task_woken);
    if(task_woken){
      portYIELD_FROM_ISR();
    }
    counter = 0;
  }
}

void task_a(void*pvParameters){
  static uint16_t sum;
  while(1){
    xSemaphoreTake(bin_sem, portMAX_DELAY);
    for(int i = 0; i < 10; i++ ){
      sum += (buf_task[i]);
    }
    Serial.println(sum / 10);
    memset(buf_task, NULL, 10);
    sum = 0;
  }
}

void task_b(void *pvParameters){
  static const uint8_t buf_len = 200;
  uint8_t counter;
  static char buffer[buf_len];
  char c;
  while(1){
    while(Serial.available() > 0){
      c = Serial.read();
      if(c == '\n'){
        Serial.println(buffer);
        memset(buffer, NULL, buf_len);
        counter = 0;
      }
      else{
        if(counter < buf_len - 1){
          buffer[counter] = c;
          counter ++;
        }
      }
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
