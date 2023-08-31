int8_t motor_init = 21;
int8_t motor_forw = 23;
int8_t motor_back = 22;

uint8_t i = 0;

const uint16_t freq = 30000;
const uint8_t pwmChannel1 = 0;
const uint8_t resolution = 8;

void motor_speed(uint8_t channel, uint8_t speed) {
  ledcWrite(channel, speed);
}

void setup() {
  pinMode(motor_forw, OUTPUT);
  pinMode(motor_back, OUTPUT);

  digitalWrite(motor_forw, HIGH);
  digitalWrite(motor_back, LOW);

  ledcSetup(pwmChannel1, freq, resolution);
  ledcAttachPin(motor_init, pwmChannel1);
}

void loop() {
  for (i = 0; i < 256; i++) {
    motor_speed(0, i);
    delay(100);
  }
}
