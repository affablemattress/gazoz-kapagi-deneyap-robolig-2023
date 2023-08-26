void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  attachInterrupt(digitalPinToInterrupt(18), blink, FALLING);
}

uint8_t counter;

void blink(){
  counter ++;
}

void loop() {
  digitalWrite(LED_BUILTIN, counter % 2);
}
