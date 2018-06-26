#include "QTRSensors.h"
#include "SensorsHelper.h"
#include "Constants.h"
#include <Arduino.h>
#include "Pins.h"

unsigned int frontalSensorsValues[NUM_SENSORS];

QTRSensorsRC frontalSensors((unsigned char[]) {
  2, A5, A4, A3, A2, A1, A0, 4
}, NUM_SENSORS, 1000);

QTRSensorsRC rightSensor((unsigned char[]) {
  RIGHT_SENSOR_PIN
}, 1, 3000);


void calibrateSensors()
{
  frontalSensors.calibrate();
  rightSensor.calibrate();
}

void resetCalibration()
{
  frontalSensors.resetCalibration();
  rightSensor.resetCalibration();
}

float readArray()
{
  int line = frontalSensors.readLine(frontalSensorsValues);
  return (line - ARRAY_CENTER_POSITION) / 1000;
}

bool readRight()
{
  unsigned int rightValue;
  rightSensor.readCalibrated(&rightValue);
  return rightValue < LINE_VALUE;
}
