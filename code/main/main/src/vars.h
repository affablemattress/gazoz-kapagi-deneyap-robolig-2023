#pragma once


//<------------------------------------------------------------------------------>
//<-------------------------------------VARS------------------------------------->
//<------------------------------------------------------------------------------>
#define DEBOUNCE_CONSTANT 150

//TODO find angles
#define GRIP_RELAX_ANGLE 0
#define GRIP_HOLD_ANGLE 180
#define TRIG_DEF_ANGLE 0
#define TRIG_PRESSED_ANGLE 180

#define SONG_CHANNEL 4

//<------------------------------------------------------------------------------>
//<--------------------------------------PINS------------------------------------>
//<------------------------------------------------------------------------------>
#define BUT1 D0
#define BUT2 D1

#define LEDG A4
#define LEDB A5

#define PWML D4
#define PWMR SCK

#define IN1 MISO
#define IN2 MOSI
#define IN3 D8
#define IN4 D9

#define BUZZ D15

#define SV_GRIP DAC1
#define SV_TRIG DAC2
