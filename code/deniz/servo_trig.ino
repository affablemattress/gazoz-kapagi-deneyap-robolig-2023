#include <Deneyap_Servo.h>

Servo servo_ok_tutan;
Servo ok_trigger;
void setup() {
  servo_ok_tutan.attach(, 2);  // Pin
  ok_trigger.attach(, 3);      // Pin
  servo_ok_tutan.write(); //init position
  ok_trigger.write(); //init position
}

void loop() {
  
}

void ok_front_open(){
  servo_ok_tutan.write(); // önü aç
}
void ok_fire(){
  ok_trigger.write(); // ateşle 
}
