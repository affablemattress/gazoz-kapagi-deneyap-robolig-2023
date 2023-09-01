#include "vars.h"
#include "esp_now.h"
#include "WiFi.h"
#include <Arduino.h>

#define DEBUGa

struct ControllerData {
  volatile int16_t analogX;
  volatile int16_t analogY;
  volatile uint8_t baseServoPosition;
  volatile uint8_t hitServoPosition;
};
ControllerData myControllerData = {
  .analogX = 0,
  .analogY = 0,
  .baseServoPosition = 150,
  .hitServoPosition = HIT_SERVO_LOW
};

esp_now_peer_info_t mainPeerInfo = {
  .peer_addr = { 0xA8, 0x42, 0xE3, 0x89, 0x63, 0xD4 },
  .lmk = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
  .channel = 0,
  .ifidx = WIFI_IF_STA,
  .encrypt = 0,
  .priv = NULL
};

void interruptLB();
void interruptCB();
void interruptRB();

void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println(WiFi.macAddress());
  #endif

//<------------------------------------------------------------------------------>
//<----------------------------------SETUP PINS---------------------------------->
//<------------------------------------------------------------------------------>
  pinMode(LB, INPUT_PULLUP);
  pinMode(CB, INPUT_PULLUP);
  pinMode(RB, INPUT_PULLUP);

  pinMode(MY_LEDB, OUTPUT);
  digitalWrite(MY_LEDB, 1);
  pinMode(MY_LEDG, OUTPUT);

  pinMode(JOYX, INPUT);
  pinMode(JOYY, INPUT);

  pinMode(BUZZ, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(LB), interruptLB, FALLING);
  attachInterrupt(digitalPinToInterrupt(CB), interruptCB, FALLING);
  attachInterrupt(digitalPinToInterrupt(RB), interruptRB, FALLING);

//<------------------------------------------------------------------------------>
//<-------------------------------SETUP ESP_NOW---------------------------------->
//<------------------------------------------------------------------------------>
  WiFi.mode(WIFI_STA);
  vTaskDelay(1000);
  while(esp_now_init() != 0) {
    #ifdef DEBUG
    Serial.println("ESP_NOW setup failed. Retrying in 500ms...");
    #endif
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  //esp_now_set_pmk(pmk);
  esp_now_add_peer(&mainPeerInfo);
  #ifdef DEBUG
  Serial.println("ESP_NOW setup successful!");
  #endif
}

void loop() {
  #ifdef DEBUG
  Serial.print("ANALOG X, ANALOG Y: ");
  Serial.print(rollingAvgX);
  Serial.print(" , ");
  Serial.println(rollingAvgY);
  #endif

  int16_t analogX = -analogRead(JOYX);
  int16_t analogY = -analogRead(JOYY);

  myControllerData.analogX = analogX - 5120;
  myControllerData.analogY = analogY - 5120;

  if(myControllerData.analogX  > -100 && myControllerData.analogX  < 100) {
    myControllerData.analogX  = 0;
  }
  if(myControllerData.analogY  > -100 && myControllerData.analogY  < 100) {
    myControllerData.analogY  = 0;
  }

  if(myControllerData.analogX  >= 100 && myControllerData.analogX  < 6800) {
    myControllerData.analogX  = MOTOR_LOW_SPEED;
  } else if(myControllerData.analogX  > -6800 && myControllerData.analogX  <= -100) {
    myControllerData.analogX  = -MOTOR_LOW_SPEED;
  }
  if(myControllerData.analogY >= 100 && myControllerData.analogY < 6800) {
    myControllerData.analogY = MOTOR_LOW_SPEED;
  } else if(myControllerData.analogY  > -6800 && myControllerData.analogY  <= -100) {
    myControllerData.analogY = -MOTOR_LOW_SPEED;
  }

  if(myControllerData.analogX >= 6800) {
    myControllerData.analogX  = 1000;
  } else if(myControllerData.analogX  <= -6800) {
    myControllerData.analogX  = -1000;
  }
  if(myControllerData.analogY >= 6800) {
    myControllerData.analogY = 1000;
  } else if(myControllerData.analogY <= -6800) {
    myControllerData.analogY = -1000;
  }

  esp_now_send(mainPeerInfo.peer_addr, (uint8_t *) &myControllerData, sizeof(myControllerData));

  #ifdef DEBUG
  static char printBuffer[100];
  sprintf(printBuffer, "Analog X: %d - Analog Y: %d - Left Button: %u - Right Button: %u\n", myControllerData.analogX, myControllerData.analogY, myControllerData.leftButton, myControllerData.rightButton);
  Serial.print(printBuffer);
  #endif
  vTaskDelay(35 / portTICK_PERIOD_MS);
}

void interruptLB() {
  volatile static uint32_t isr_db_counter = 0;
  if (millis() - isr_db_counter > DEBOUNCE_CONSTANT) {
    if(myControllerData.baseServoPosition < 180) {
      myControllerData.baseServoPosition += 10;
    }
    isr_db_counter = millis();
  }
}

void interruptCB() {
  volatile static uint32_t isr_db_counter = 0;
  if (millis() - isr_db_counter > DEBOUNCE_CONSTANT) {
    if (myControllerData.hitServoPosition == HIT_SERVO_LOW) {
      myControllerData.hitServoPosition = 180;
    } else {
      myControllerData.hitServoPosition = HIT_SERVO_LOW;
    }
    isr_db_counter = millis();
  }
}

void interruptRB() {
  volatile static uint32_t isr_db_counter = 0;
  if (millis() - isr_db_counter > DEBOUNCE_CONSTANT) {
    if(myControllerData.baseServoPosition > 0) {
      myControllerData.baseServoPosition -= 10;
    }
    isr_db_counter = millis();
  }
}
