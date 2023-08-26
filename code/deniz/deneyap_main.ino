//Libraries
#include <Deneyap_Servo.h>
#include <tcs3200.h>
#include <esp_now.h>
#include <WiFi.h>
// Data
typedef struct dataPack {
  uint16_t X_axis;
  uint16_t Y_axis;
  uint8_t General_state = 0;    // read lines 1 robotic arm 2 color 3 etc
  uint8_t Grab_Drop_state = 0;  // 0 drop 1 grab

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


//Motor sürücü chunk
uint8_t motor_1_pwm = D0;
uint8_t motor_1_forward = D6;
uint8_t motor_1_backward = D7;

uint8_t motor_2_pwm = D1;
uint8_t motor_2_forward = D8;
uint8_t motor_2_backward = D9;


const uint16_t freq = 30000;
const uint8_t pwmChannel = 0;
const uint8_t resolution = 8;
uint8_t speed = 200;  // between 0 255

void motor_speed(uint8_t motor_pin, uint8_t speed);
void motor_way(uint8_t motor_pin, uint8_t motor_forw_back);  //forward 1 backward 0

//Deneyap analog and digital pins
uint8_t i[] = { A0, A1, A2, A3, A4, A5 };
uint8_t j[] = { D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13, D14, D15 };
uint8_t state = 0;
uint16_t analogDump[] = { 0, 0 };
uint8_t digital5 = 0;


//Classes and structs for devices
Servo servo_D2;
Servo servo_D3;  //Robot kol için
Servo servo_D4;

//prototypes
void robot_arm_controller(int16_t angle_1, int16_t angle_2, bool grab_drop);
void line_tracer_code();  // Kayra sen bunu halledersin ya da birlikte kafa yorabiliriz

//İnitialisation
void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);//ESP NOW
 
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);

  //Motor init
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(motor_1_pwm, pwmChannel);
  ledcAttachPin(motor_2_pwm, pwmChannel);
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
  pinMode(D8, OUTPUT);
  pinMode(D9, OUTPUT);

  //Robot kol init
  servo_D2.attach(i[2], 1, 50);
  servo_D3.attach(i[3], 1, 50);
  servo_D4.attach(i[4], 1, 50);
  pinMode(D5, INPUT);
}

//code block NOTE => may not used due to ethernet connection
void loop() {
  analogDump[0] = analogRead(i[0]);
  analogDump[1] = analogRead(i[1]);
  digital5 = digitalRead(D5);
  Red = tcs.colorRead('r');
  Green = tcs.colorRead('g');
  Blue = tcs.colorRead('b');
}


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

void motor_speed(uint8_t motor_pin, uint8_t speed) {
  ledcWrite(motor_pin, speed);
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
