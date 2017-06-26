#ifndef Pins_h
#define Pins_h

#include <Arduino.h>

// Sensores de reflectancia
unsigned char SensorArrayPins[] = { A4, A3, A2, A1 };
unsigned char LeftBorderSensorPin = 6;
unsigned char RightBorderSensorPin = 7;

#define COMMAND_BUTTON_PIN  5
#define STATUS_LED_PIN 13
#define KILLSWITCH_PIN 8

//Pinos do Microcontrolador
#define LEFT_MOTOR_PIN_1 3
#define LEFT_MOTOR_PIN_2 11
#define RIGHT_MOTOR_PIN_1 10
#define RIGHT_MOTOR_PIN_2 9
#define BUZZER_PIN 12
#define ENCODER_PIN 2

#endif //Pins_h
