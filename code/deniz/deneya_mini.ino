#include <esp_now.h>
#include <WiFi.h>


uint8_t analog_digital_state = 0;
int8_t speed = 0;
int8_t right_left_state = 0;

//ESP NOW
uint8_t broadcastAddress[] = { 0x3C, 0xE9, 0x0E, 0x86, 0x0C, 0x84 };
esp_now_peer_info_t peerInfo;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

typedef struct dataPack {
  uint16_t X_axis = 0;      // dummy number for test
  uint16_t Y_axis = 0;      //dummy number for test
  uint8_t an_dig_state = 0;  // button 0 - joystick 1
  int8_t f_b_speed = 0;     //speed -100 100
  int8_t horizontal = 0;    //-1 right 0 forward 1 left

} dataPack;

dataPack instance;

//Interrupt Functions

void speed_inc();
void speed_dec();
void turn_right();
void turn_left();
void analog_digital();

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);  //ESP NOW

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }


  pinMode(23, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);

  attachInterrupt(23,speed_inc, FALLING);
  attachInterrupt(22,speed_dec , FALLING);
  attachInterrupt(1,turn_right , FALLING);
  attachInterrupt(3,turn_left , FALLING);
  attachInterrupt(21,analog_digital , FALLING);
}

void loop() {
  if(analog_digital_state==1){
    instance.X_axis= analogRead(34);
    instance.X_axis= analogRead(35);
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

  delay(100);
}
void speed_inc() {
  if (speed > 90) {
    speed = 100;
  } else {
    speed += 10;
  }
  instance.f_b_speed = speed;
}
void speed_dec() {
  if (speed < -90) {
    speed = -100;
  } else {
    speed -= 10;
  }
  instance.f_b_speed = speed;
}
void turn_right() {
  if (right_left_state == 1) {
    right_left_state = 1;
  } else {
    right_left_state += 1;
  }
  instance.horizontal = right_left_state;
}
void turn_left() {
  if (right_left_state == -1) {
    right_left_state = -1;
  } else {
    right_left_state -= 1;
  }
  instance.horizontal = right_left_state;
}
void analog_digital() {
  analog_digital_state += 1;
  analog_digital_state %= 2;
  instance.an_dig_state = analog_digital_state;
}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}