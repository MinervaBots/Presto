#include "QTRSensors.h"
#include "SensorsHelper.h"
#include "Constants.h"
#include <Arduino.h>
#include "Pins.h"

unsigned int Sensors[NUM_SENSORS];
unsigned long lastReadLeft, lastReadRight;
unsigned long rightCount = 0;

QTRSensorsRC frontalSensors((unsigned char[]) {
  2, A5, A4, A3, A2, A1, A0, 4
}, NUM_SENSORS, 1000);
QTRSensorsRC rightSensor((unsigned char[]) {
  RIGHT_SENSOR_PIN
}, 1, 3000);
QTRSensorsRC leftSensor((unsigned char[]) {
  LEFT_SENSOR_PIN
}, 1, 3000);

void calibrateSensors()
{
  frontalSensors.calibrate();
  rightSensor.calibrate();
  leftSensor.calibrate();
}

float readArray()
{
  int line = frontalSensors.readLine(Sensors);
  return (line - ARRAY_CENTER_POSITION) / 1000;
}

bool readRight(unsigned long maxCount)
{
  unsigned int rightValue;
  rightSensor.readCalibrated(&rightValue);
  
  if (millis() - lastReadRight > RIGHT_SENSOR_LOW_TIME)
  {
    if (rightValue < LINE_VALUE)
    {
      rightCount++;
      digitalWrite(LED_PIN, rightCount % 2 == 0 ? HIGH : LOW);
      lastReadRight = millis();
    }
  }
  return rightCount >= maxCount;
}

void resetRightCount()
{
  rightCount = 0;
}

bool readLeft()
{
  unsigned int leftValue;
  leftSensor.readCalibrated(&leftValue);

  if (millis() - lastReadLeft > LEFT_SENSOR_LOW_TIME)
  {
    if (leftValue < LINE_VALUE)
    {
      return true;
      lastReadLeft = millis();
    }
  }
  return false;
}


