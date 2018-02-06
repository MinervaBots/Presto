#include <Arduino.h>

#ifndef SETTINGS_H
#define SETTINGS_H

#define BUTTON 0 // verify pin
#define LED_1 1 // verify pin
#define LED_2 2 // verify pin
#define LED_3 3 // verify pin
#define LEFT_MOTOR_PWM 4 // verify pin
#define LEFT_MOTOR_P1 5 // verify pin
#define LEFT_MOTOR_P2 6 // verify pin
#define RIGHT_MOTOR_PWM 7 // verify pin
#define RIGHT_MOTOR_P1 8 // verify pin
#define RIGHT_MOTOR_P2 9 // verify pin 
#define LEFT_MARK 10 // verify pin
#define RIGT_MARK 11 // verify pin

#define ARRAY_SIZE 3 // changes depending on the array
const unsigned char array_pins[] = {12, 13, 14}; // verify pins

#define KP_SAFE 1
#define KI_SAFE 0
#define KD_SAFE 0
#define SPEED_SAFE 0.5

#define KP_STRAIGHT 1
#define KI_STRAIGHT 0
#define KD_STRAIGHT 0
#define SPEED_STRAIGHT 1

#define KP_TURN 1
#define KI_TURN 0
#define KD_TURN 0
#define SPEED_TURN 0.5
#define OFFSET_TURN 1

void mySetup() {
	pinMode(,INPUT);
}

#endif
