#include "QTRSensors.h"
#include "SensorsHelper.h"
#include "Constants.h"
#include <Arduino.h>
#include "Pins.h"

unsigned int arraySensorsValue[NUM_SENSORS];

QTRSensorsAnalog frontalSensors((unsigned char[]){
  A8, A7, A6, A5, A4, A3, A2, A1, A0, A20, A22
}, NUM_SENSORS/*, 4, 255*/);   // 4(numSamples) and 255(emitterPin) are default     

QTRSensorsAnalog rightSensor((unsigned char[]){
  RIGHT_SENSOR_PIN
}, 1);

QTRSensorsAnalog leftSensor((unsigned char[]){
  LEFT_SENSOR_PIN
}, 1);

void calibrateSensors()
{
  frontalSensors.calibrate();
  rightSensor.calibrate();
  leftSensor.calibrate();
}

void resetCalibration()
{
  frontalSensors.resetCalibration();
  rightSensor.resetCalibration();
  leftSensor.resetCalibration();
}

float readArray()
{
  int line = frontalSensors.readLine(arraySensorsValue);
  return (line - ARRAY_CENTER_POSITION)/1000.0;
}

bool readRight()
{
  unsigned int rightValue;
  rightSensor.readCalibrated(&rightValue);
  return rightValue < LINE_VALUE;
}

bool readLeft()
{
  unsigned int leftValue;
  leftSensor.readCalibrated(&leftValue);
  return leftValue < LINE_VALUE;
}
