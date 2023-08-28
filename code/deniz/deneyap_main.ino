//Libraries
#include <Deneyap_Servo.h>
#include <tcs3200.h>
#include <esp_now.h>
#include <WiFi.h>

// Data
int8_t F_B_axis;
int8_t R_L_axis;
int8_t R_motor;
int8_t L_motor;

typedef struct dataPack {
  uint16_t X_axis;       // dummy number for test
  uint16_t Y_axis;       //dummy number for test
  uint8_t an_dig_state;  // button 0 - joystick 1
  int8_t f_b_speed = 0;  //speed -100 100
  int8_t horizontal;     //-1 right 0 forward 1 left
} dataPack;

dataPack instance;

//tcs3200 renk okuma
tcs3200 tcs(D11, D12, D13, D14, D15);

/*RGB values*/
uint8_t Red = 0;
uint8_t Green = 0;
uint8_t Blue = 0;
uint8_t h = 0;
uint8_t s = 0;
uint8_t v = 0;
void RgbToHsv(uint8_t red, uint8_t green, uint8_t blue, double& hue, double& saturation, double& value);
bool fnc_tcs34725_iscolor(int h, int _color);


//Motor sürücü chunk
uint8_t motor_1_pwm = D0;
uint8_t motor_1_forward = D6;
uint8_t motor_1_backward = D7;

uint8_t motor_2_pwm = D1;
uint8_t motor_2_forward = D8;
uint8_t motor_2_backward = D9;


const uint16_t freq = 2000;
const uint8_t pwmChannel1 = 0;
const uint8_t pwmChannel2 = 1;

void motor_speed(uint8_t motor_pin, uint8_t speed);          // Problematic due to channel and ledcwrite
void motor_way(uint8_t motor_pin, uint8_t motor_forw_back);  //forward 1 backward 0

//Deneyap analog and digital pins
uint8_t i[] = { A0, A1, A2, A3, A4, A5 };
uint8_t j[] = { D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13, D14, D15 };


//Classes and structs for devices
Servo servo_D2;
Servo servo_D3;  //Robot kol için
Servo servo_D4;

//prototypes
void robot_arm_controller(int16_t angle_1, int16_t angle_2, bool grab_drop);
void line_tracer_code();  // Kayra sen bunu halledersin ya da birlikte kafa yorabiliriz. Nope bu yok

//İnitialisation
void setup() {
  Serial.begin(115200);

  //ESP NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  //Motor init
  ledcSetup(pwmChannel1, freq, resolution);
  ledcAttachPin(motor_1_pwm, pwmChannel);
  ledcSetup(pwmChannel2, freq, resolution);
  ledcAttachPin(motor_2_pwm, pwmChannel);
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
  pinMode(D8, OUTPUT);
  pinMode(D9, OUTPUT);
  //Motor way init
  motor_way(D0);
  motor_way(D1);

  //Robot kol init
  servo_D2.attach(, 1, 50);
  servo_D3.attach(, 1, 50);
  servo_D4.attach(, 1, 50);
  pinMode(D5, INPUT);
}


void loop() {
  if (instance.an_dig_state == 0) {
    temp_speed = map(instance.f_b_speed, , 100, ) if (instance.horizontal == 0) {
      if (!((digitalRead(motor_1_forward) == HIGH) && (digitalRead(motor_2_forward) == HIGH))) {
        motor_way(D0);
        motor_way(D1);
      }

      motor_speed(0, instance.f_b_speed);
      motor_speed(1, instance.f_b_speed);
    }
    else if (instance.horizontal == 1) {
      if (!((digitalRead(motor_1_forward) == HIGH) && (digitalRead(motor_2_forward) == LOW))) {
        motor_way(D0, 1);
        motor_way(D1, 0);
      }
      motor_speed(0, 100);
      motor_speed(1, 100);
    }
    else if (instance.horizontal == -1) {
      if (!((digitalRead(motor_1_forward) == HIGH) && (digitalRead(motor_2_forward) == LOW))) {
        motor_way(D0, 0);
        motor_way(D1, 1);
      }
      motor_speed(0, 100);
      motor_speed(1, 100);
    }
  } else if (instance.an_dig_state == 1) {
    //motor hızlarını rescale ediyoruz
    F_B_axis = map(instance.X_axis, 0, 4095, -60, 60);  //düz hız
    R_L_axis = map(instance.X_axis, 0, 4095, -60, 60);  // sağ sola göre motor hızını değiştiren parametre
    if (R_L_axis < 0) {
      L_motor = F_B_axis + R_L_axis; //Sağ motor
      R_motor = F_B_axis - R_L_axis; // Sol motor
      if (L_motor >= 0) {
        motor_way(D0);
      } else {
        motor_way(D0, 0);
      }
      if (R_motor >= 0) {
        motor_way(D1);
      } else {
        motor_way(D1, 0);
      }
    } else {
      L_motor = F_B_axis + R_L_axis;
      R_motor = F_B_axis - R_L_axis;
      if (L_motor >= 0) {
        motor_way(D0);
      } else {
        motor_way(D0, 0);
      }
      if (R_motor >= 0) {
        motor_way(D1);
      } else {
        motor_way(D1, 0);
      }
    }
    motor_speed(0,abs(R_speed));
    motor_speed(1,abs(L_speed));
  }
}

//Belki kullanılabilir.
void robot_arm_controller(int16_t angle_1, int16_t angle_2, bool grab_drop) {
  angle_1 = map(angle_1, 0, 4095, 0, 180);
  angle_2 = map(angle_1, 0, 4095, 0, 180);
  servo_D2.write(angle_1);
  servo_D3.write(angle_2);
  if (grab_drop) {
    servo_D4.write(180);
  } else {
    servo_D4.write(0);
  }
}

void motor_speed(uint8_t channel, uint8_t speed) {
  ledcWrite(channel, speed);
}

void motor_way(uint8_t motor_pin, uint8_t motor_forw_back = 1) {
  if (motor_pin == D0) {
    if (motor_forw_back == 1) {
      digitalWrite(motor_1_forward, HIGH);
      digitalWrite(motor_1_backward, LOW);
    } else if (motor_forw_back == 0) {
      digitalWrite(motor_1_forward, LOW);
      digitalWrite(motor_1_backward, HIGH);
    }
  } else if (motor_pin == D1) {
    if (motor_forw_back == 1) {
      digitalWrite(motor_2_forward, HIGH);
      digitalWrite(motor_2_backward, LOW);
    } else if (motor_forw_back == 0) {
      digitalWrite(motor_2_forward, LOW);
      digitalWrite(motor_2_backward, HIGH);
    }
  }
}

void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  memcpy(&instance, incomingData, sizeof(instance));
}

void RgbToHsv(uint8_t red, uint8_t green, uint8_t blue, double& hue, double& saturation, double& value) {
  float rd = (float)red / 255;
  float gd = (float)green / 255;
  float bd = (float)blue / 255;
  float maximum = max(max(rd, gd), bd), minimum = min(min(rd, gd), bd);

  value = maximum;

  float d = maximum - minimum;
  saturation = (maximum == 0) ? 0 : d / maximum;

  hue = 0;
  if (maximum != minimum) {
    if (maximum == rd) {
      hue = (gd - bd) / d + (gd < bd ? 6 : 0);
    } else if (maximum == gd) {
      hue = (bd - rd) / d + 2;
    } else if (maximum == bd) {
      hue = (rd - gd) / d + 4;
    }
    hue /= 6;
  }
}

bool fnc_tcs34725_iscolor(int h, int _color) {

  if (h > 340 || h < 20) {
    if (_color == 2) return true;
  }  //red
  else if (h < 45) {
    if (_color == 3) return true;
  }  //orange
  else if (h < 70) {
    if (_color == 4) return true;
  }  //yellow
  else if (h < 150) {
    if (_color == 5) return true;
  }  //green
  else if (h < 210) {
    if (_color == 6) return true;
  }  //cyan
  else if (h < 265) {
    if (_color == 7) return true;
  }  //blue
  else if (h < 340) {
    if (_color == 8) return true;
  }  //violet
  return false;
}
