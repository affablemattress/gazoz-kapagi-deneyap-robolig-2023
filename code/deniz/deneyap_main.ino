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
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len);
dataPack instance;

//tcs3200 renk okuma
//tcs3200 tcs(D11, D12, D13, D14, D15);

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
uint8_t m_1_1pwm = 16;
uint8_t m_1_1f = 2;
uint8_t m_1_1b = 4;

uint8_t m_1_2pwm = 17;
uint8_t m_1_2f = 36;
uint8_t m_1_2b = 34;

uint8_t m_2_1pwm = 23;
uint8_t m_2_1f = 5;
uint8_t m_2_1b = 13;

uint8_t m_2_2pwm = 27;
uint8_t m_2_2f = 12;
uint8_t m_2_2b = 14;


const uint16_t freq = 1000;
const uint8_t pwmChannel1 = 0;
const uint8_t pwmChannel2 = 1;
const uint8_t resolution = 10;

void motor_speed(uint8_t motor_pin, uint8_t speed);  // Problematic due to channel and ledcwrite
void m_r_f();
void m_l_f();
void m_r_b();
void m_l_b();

//Deneyap analog and digital pins
/*
uint8_t i[] = { A0, A1, A2, A3, A4, A5 };
uint8_t j[] = { D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13, D14, D15 };


//Classes and structs for devices
Servo servo_D2;
Servo servo_D3;  //Robot kol için
Servo servo_D4;
*/
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
  ledcAttachPin(m_1_1pwm, pwmChannel1);
  ledcAttachPin(m_2_1pwm, pwmChannel1);
  ledcSetup(pwmChannel2, freq, resolution);
  ledcAttachPin(m_1_2pwm, pwmChannel2);
  ledcAttachPin(m_2_2pwm, pwmChannel2);

  //way pins
  pinMode(m_1_1f, OUTPUT);
  pinMode(m_1_1b, OUTPUT);
  pinMode(m_1_2f, OUTPUT);
  pinMode(m_1_2b, OUTPUT);
  pinMode(m_2_1f, OUTPUT);
  pinMode(m_2_1b, OUTPUT);
  pinMode(m_2_2f, OUTPUT);
  pinMode(m_2_2b, OUTPUT);
  //Motor way init
  m_r_f();
  m_l_f();
  //Robot kol init
  /*servo_D2.attach(, 1, 50);
  servo_D3.attach(, 1, 50);
  servo_D4.attach(, 1, 50);
  */
}


void loop() {
  if (instance.an_dig_state == 0) {
    if (instance.horizontal == 0) {
      m_r_f();
      m_l_f();

      motor_speed(0, abs(instance.f_b_speed));
      motor_speed(1, abs(instance.f_b_speed));
    } else if (instance.horizontal == 1) {
      m_r_f();
      m_l_b();

      motor_speed(0, 100);
      motor_speed(1, 100);
    } else if (instance.horizontal == -1) {
      m_r_b();
      m_l_f();

      motor_speed(0, 100);
      motor_speed(1, 100);
    }
  } else if (instance.an_dig_state == 1) {
    //motor hızlarını rescale ediyoruz
    F_B_axis = map(instance.X_axis, 0, 4095, -60, 60);  //düz hız
    R_L_axis = map(instance.X_axis, 0, 4095, -60, 60);  // sağ sola göre motor hızını değiştiren parametre

    L_motor = F_B_axis + R_L_axis;  //Sağ motor
    R_motor = F_B_axis - R_L_axis;  // Sol motor
    if (L_motor >= 0) {
      m_l_f();
    } else {
      m_l_b();
    }
    if (R_motor >= 0) {
      m_r_f();
    } else {
      m_r_b();
    }
    motor_speed(0, abs(R_motor));
    motor_speed(1, abs(L_motor));
  }
}

/*
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
*/
void motor_speed(uint8_t channel, uint8_t speed) {
  ledcWrite(channel, speed);
}
void m_r_f() {
  digitalWrite(m_1_1f, HIGH);
  digitalWrite(m_2_1f, HIGH);
  digitalWrite(m_1_1b, LOW);
  digitalWrite(m_2_1b, LOW);
}
void m_l_f() {
  digitalWrite(m_1_2f, HIGH);
  digitalWrite(m_2_2f, HIGH);
  digitalWrite(m_1_2b, LOW);
  digitalWrite(m_2_2b, LOW);
}
void m_r_b() {
  digitalWrite(m_1_1f, LOW);
  digitalWrite(m_2_1f, LOW);
  digitalWrite(m_1_1b, HIGH);
  digitalWrite(m_2_1b, HIGH);
}
void m_l_b() {
  digitalWrite(m_1_2f, LOW);
  digitalWrite(m_2_2f, LOW);
  digitalWrite(m_1_2b, HIGH);
  digitalWrite(m_2_2b, HIGH);
}
/*
void motor_way(uint8_t motor_pin, uint8_t motor_forw_back) {
  if (motor_pin == D0) {
    if (motor_forw_back == 1) {
      digitalWrite(motor_1_forward, HIGH);
      digitalWrite(motor_1_backward, LOW);
      digitalWrite(motor_1_forward, HIGH);
      digitalWrite(motor_1_backward, LOW);
    } else if (motor_forw_back == 0) {
      digitalWrite(motor_1_forward, LOW);
      digitalWrite(motor_1_backward, HIGH);
      digitalWrite(motor_1_forward, LOW);
      digitalWrite(motor_1_backward, HIGH);
    }
  } else {
    if (motor_forw_back == 1) {
      digitalWrite(motor_1_forward, LOW);
      digitalWrite(motor_1_backward, HIGH);
      digitalWrite(motor_1_forward, LOW);
      digitalWrite(motor_1_backward, HIGH);
    } else if (motor_forw_back == 0) {
      digitalWrite(motor_2_forward, LOW);
      digitalWrite(motor_2_backward, HIGH);
      digitalWrite(motor_2_forward, LOW);
      digitalWrite(motor_2_backward, HIGH);
    }
  }
}
*/
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
