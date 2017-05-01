#include "QTRSensors.h"

const int encoder_pin = 10;
QTRSensorsRC encoder((unsigned char[]) { encoder_pin }, 1);

void setup()
{
  delay(500);
  pinMode(13, HIGH);

  for (int i = 0; i < 400; i++)
  {
    encoder.calibrate();
  }
  
  digitalWrite(13, LOW);
  Serial.begin(9600);
}

void loop()
{
  unsigned int reading[1];
  encoder.readCalibrated(reading);
  Serial.println(reading[0]);
}
