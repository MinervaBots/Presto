#include "QTRSensors.h"
#include "SensorsHelper.h"
#include "Constants.h"
#include <Arduino.h>
#include "Pins.h"

unsigned int frontalSensorsValues[NUM_SENSORS];
unsigned int lateralSensorsValues[2];
unsigned long lastReadLaterals, lastReadLeft, lastReadRight;
unsigned long rightCount = 0;

QTRSensorsRC frontalSensors((unsigned char[]) {
  2, A5, A4, A3, A2, A1, A0, 4
}, NUM_SENSORS, 1000);

/*
QTRSensorsRC lateralSensors((unsigned char[]) {
  LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN
}, 2, 1000);
*/

QTRSensorsRC rightSensor((unsigned char[]) {
  RIGHT_SENSOR_PIN
}, 1, 3000);

QTRSensorsRC leftSensor((unsigned char[]) {
  LEFT_SENSOR_PIN
}, 1, 3000);

void calibrateSensors()
{
  frontalSensors.calibrate();
  //lateralSensors.calibrate();
  
  rightSensor.calibrate();
  leftSensor.calibrate();
}

float readArray()
{
  bool on_line;
  int line = frontalSensors.readLine(frontalSensorsValues, &on_line);
  return (line - ARRAY_CENTER_POSITION) / 1000;
}
/*
void readLaterals(bool *left, bool* right)
{
  if(millis() - lastReadLaterals < 100)
  {
    return;
  }
  lastReadLaterals = millis();
  
  bool on_line;
  int line = lateralSensors.readLine(lateralSensorsValues, &on_line);
  
  if(!on_line || abs(line - 500) < 200)
  {
    *left = false;
    *right = false;
  }
  else if(line < 500)
  {
    *left = true;
    *right = false;
  }
  else if(line > 500)
  {
    *left = false;
    *right = true;
  }
  
  Serial.print(on_line);
  Serial.print("\t");
  Serial.print(*left);
  Serial.print("\t");
  Serial.println(*right);
  
  Serial.print(line);
  Serial.print("\t");
  Serial.print(lateralSensorsValues[0]);
  Serial.print("\t");
  Serial.println(lateralSensorsValues[1]);
}
*/

bool readRight()
{
  unsigned int rightValue;
  rightSensor.readCalibrated(&rightValue);
  if (millis() - lastReadRight > RIGHT_SENSOR_LOW_TIME)
  {
    if (rightValue < LINE_VALUE)
    {
      lastReadRight = millis();
      return true;
    }
  }
  return false;
}

bool readLeft()
{
  unsigned int leftValue;
  leftSensor.readCalibrated(&leftValue);
  if (millis() - lastReadLeft > LEFT_SENSOR_LOW_TIME)
  {
    if (leftValue < LINE_VALUE)
    {
      lastReadLeft = millis();
      return true;
    }
  }
  return false;
}
