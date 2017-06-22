#include <Arduino.h>

#define DEBUG
//#define DEBUG2


#define SENSOR_DATA_IN_PIN A1
#define ENCODER_TICK_OUT_PIN 2
#define TICK_CONTRAST 50

bool state;
int lastRead;
#ifdef DEBUG
unsigned int cnt;
#endif

void setup()
{
#ifdef DEBUG
  Serial.begin(9600);
#endif
  pinMode(SENSOR_DATA_IN_PIN, INPUT);
  pinMode(ENCODER_TICK_OUT_PIN, OUTPUT);
}

void loop()
{
  int value = analogRead(SENSOR_DATA_IN_PIN);
  int dif = abs(value - lastRead);
#ifdef DEBUG2
  Serial.println(dif);
#endif

  bool changed = dif > TICK_CONTRAST;
#ifdef DEBUG2
  Serial.println(changed);
#endif
  lastRead = value;
  if(changed)
  {
    state = !state;
#ifdef DEBUG
    cnt++;
    Serial.println(cnt);
#endif
    delay(2);
  }
  digitalWrite(ENCODER_TICK_OUT_PIN, state ? HIGH : LOW);
}
