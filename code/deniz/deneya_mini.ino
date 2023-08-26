#include <esp_now.h>
#include <WiFi.h>

int8_t i[] = { D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13, D14, D15 };
int8_t j[] = { A0, A1, A2, A3, A4, A5 };
int8_t state = 0;
int8_t grab_drop = 0;

//ESP NOW
uint8_t broadcastAddress[]={0x24, 0x6F, 0x28, 0x7A, 0xAE, 0x7C};
esp_now_peer_info_t peerInfo;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

typedef struct dataPack {
  uint16_t X_axis;
  uint16_t Y_axis;
  uint8_t General_state = 0;    // read lines 1 robotic arm 2 color 3 etc
  uint8_t Grab_Drop_state = 0;  // 0 drop 1 grab

} dataPack;

dataPack instance;

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

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  attachInterrupt(D5, state_inc, FALLING);      // Random pin
  attachInterrupt(D6, grab_drop_inc, FALLING);  // Random pin

  pinMode(D7, OUTPUT);                          //General state 0 led
  pinMode(D8, OUTPUT);                          //General state 1 led
  pinMode(D9, OUTPUT);                          //General state 2 led
  pinMode(D10, OUTPUT);                         //grab_drop state 0 led
  pinMode(D11, OUTPUT);                         //grab_drop state 1 led

  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);
  digitalWrite(D9, LOW);
  digitalWrite(D10, LOW);
  digitalWrite(D11, LOW);
}

void loop() {
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
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sending confirmed");
  }
  else {
    Serial.println("Sending error");
}


void state_inc() {
  state += 1;
  state %= 3;
  instance.General_state = state;
}
void grab_drop_inc() {
  grab_drop += 1;
  grab_drop %= 2;
  instance.Grab_Drop_state = grab_drop;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}