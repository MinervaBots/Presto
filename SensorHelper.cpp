#include "QTRSensors.h"
#include "SensorsHelper.h"
#include "Constants.h"
#include <Arduino.h>
#include "Pins.h"

unsigned int arraySensorsValue[NUM_SENSORS];

QTRSensorsAnalog frontalSensors((unsigned char[]){
 A8, A7, A6, A5, A4, A3, A2, A1, A0, A20, A22
}, NUM_SENSORS, 1/*, 255*/);   // 4(numSamples) and 255(emitterPin) are default     

QTRSensorsAnalog rightSensor((unsigned char[]){
  RIGHT_SENSOR_PIN
}, 1, 1);

QTRSensorsAnalog leftSensor((unsigned char[]){
  LEFT_SENSOR_PIN
}, 1, 1);

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
  int line = frontalSensors.readLine(arraySensorsValue, QTR_EMITTERS_ON, USE_WHITE_LINE);
  return (ARRAY_CENTER_POSITION - line)/1000.0; //invertido (ver followPath)
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

bool changeState()
{
  static unsigned long lastLeft = 0;
  if ((readLeft()) and (millis() - lastLeft > 200)) {
    lastLeft = millis();
    return true;
  }
  return false;
}
