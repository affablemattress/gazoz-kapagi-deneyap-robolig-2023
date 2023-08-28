#include <esp_now.h>
#include <WiFi.h>


uint8_t analog_digital_state = 0;
int8_t speed = 0;
int8_t right_left_state = 0;

//ESP NOW
uint8_t broadcastAddress[] = { 0x3C, 0xE9, 0x0E, 0x86, 0x0C, 0x84 };
esp_now_peer_info_t peerInfo;


typedef struct dataPack {
  uint16_t X_axis = 0;       // dummy number for test
  uint16_t Y_axis = 0;       //dummy number for test
  uint8_t an_dig_state = 0;  // button 0 - joystick 1
  int8_t f_b_speed = 0;      //speed -100 100
  int8_t horizontal = 0;     //-1 right 0 forward 1 left

} dataPack;

dataPack instance;

//Interrupt Functions

void speed_inc();
void speed_dec();
void turn_right();
void turn_left();
void analog_digital();

void setup() {
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);  //ESP NOW

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }


  pinMode(23, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  pinMode(18, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(23), speed_inc, FALLING);
  attachInterrupt(digitalPinToInterrupt(22), speed_dec, FALLING);
  attachInterrupt(digitalPinToInterrupt(21), turn_right, FALLING);
  attachInterrupt(digitalPinToInterrupt(19), turn_left, FALLING);
  attachInterrupt(digitalPinToInterrupt(18), analog_digital, FALLING);
}

void loop() {
  if (analog_digital_state == 1) {
    instance.X_axis = analogRead(34);
    instance.X_axis = analogRead(35);
  }
  /*
  instance.X_axis = analogRead(A0);
  instance.Y_axis = analogRead(A1);
  if (state == 0) {
    digitalWrite(D7, HIGH);
    digitalWrite(D9, LOW);
  } else if (state == 1) {
    digitalWrite(D8, HIGH);
    digitalWrite(D7, LOW);
    if (grab_drop) {
      digitalWrite(D10, HIGH);
      digitalWrite(D11, LOW);
    } else {
      digitalWrite(D10, LOW);
      digitalWrite(D11, HIGH);
    }
  } else if (state == 2) {
    digitalWrite(D9, HIGH);
    digitalWrite(D8, LOW);
  }
  */
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&instance, sizeof(instance));

  if (result == ESP_OK) {
    Serial.println("Sending confirmed");
  } else {
    Serial.println("Sending error");
  }
  

  delay(2000);
}


void speed_inc() {
  volatile static uint32_t isr_db_counter = 0;
  if (millis() - isr_db_counter > 150) {
    if (speed > 90) {
      speed = 100;
    } else {
      speed += 10;
    }
    instance.f_b_speed = speed;
    Serial.println(speed);
    isr_db_counter = millis();
  }
}

void speed_dec() {
  volatile static uint32_t isr_db_counter = 0;
  if (millis() - isr_db_counter > 150) {
    if (speed < -90) {
      speed = -100;
    } else {
      speed -= 10;
    }
    instance.f_b_speed = speed;
    Serial.println(speed);
    isr_db_counter = millis();
  }
}

void turn_right() {
  volatile static uint32_t isr_db_counter = 0;
  if (millis() - isr_db_counter > 150) {
    if (right_left_state == 1) {
      right_left_state = 1;
    } else {
      right_left_state += 1;
    }
    instance.horizontal = right_left_state;
    isr_db_counter = millis();
  }
}
void turn_left() {
  volatile static uint32_t isr_db_counter = 0;
  if (millis() - isr_db_counter > 150) {
    if (right_left_state == -1) {
      right_left_state = -1;
    } else {
      right_left_state -= 1;
    }
    instance.horizontal = right_left_state;
    isr_db_counter = millis();
  }
}
void analog_digital() {
  volatile static uint32_t isr_db_counter = 0;
  if (millis() - isr_db_counter > 150) {
    analog_digital_state += 1;
    analog_digital_state %= 2;
    instance.an_dig_state = analog_digital_state;
    isr_db_counter = millis();
  }
}
