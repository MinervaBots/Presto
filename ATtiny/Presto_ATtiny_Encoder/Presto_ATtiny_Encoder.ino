#include <Arduino.h>

#define SENSOR_DATA_IN_PIN 1
#define ENCODER_TICK_OUT_PIN 2
#define TICK_VALUE 200

void setup()
{
  pinMode(SENSOR_DATA_IN_PIN, INPUT);
  pinMode(ENCODER_TICK_OUT_PIN, OUTPUT);
}

void loop()
{
  digitalWrite(ENCODER_TICK_OUT_PIN, analogRead(SENSOR_DATA_IN_PIN) < TICK_VALUE ? HIGH : LOW);
}
