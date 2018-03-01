#include "QTRSensors.h"
#include "SensorsHelper.h"
#include "Constants.h"
#include <Arduino.h>
#include "Pins.h"

unsigned int frontalSensorsValues[NUM_SENSORS];
unsigned int lateralSensorsValues[2];
unsigned int lastLeftValue;
unsigned int lastRightValue;

QTRSensorsRC frontalSensors((unsigned char[]) {
  2, A5, A4, A3, A2, A1, A0, 4
}, NUM_SENSORS, 1000);

QTRSensorsRC lateralSensors((unsigned char[]) {
  LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN
}, 2, 1000);


void calibrateSensors()
{
  frontalSensors.calibrate();
  lateralSensors.calibrate();
}

void resetCalibration()
{
  frontalSensors.resetCalibration();
  lateralSensors.resetCalibration();
}

float readArray()
{
  bool on_line;
  int line = frontalSensors.readLine(frontalSensorsValues, &on_line);
  return (line - ARRAY_CENTER_POSITION) / 1000;
}



void readLaterals(bool *left, bool* right)
{
  bool on_line;
  int line = lateralSensors.readLine(lateralSensorsValues, &on_line);
  
  if(!on_line || abs(line - 500) < 200)
  {
    *left = false;
    *right = false;
  }
  else if(line < 500)
  {
    
    *left = ((lastLeftValue - lateralSensorsValues[0]) > LINE_VALUE);   // #
    *right = false;
  }
  else if(line > 500)
  {
    *left = false;
    *right = ((lastRightValue - lateralSensorsValues[1]) > LINE_VALUE); // #
  }

  // #: Considera apenas quando a leitura passa de preto (1000) para branco (0)
  // Desse forma a diferença fica algo como:
  // 1000 - 0 > LINE_VALUE
  // Igor: "tipo interrupção no rise"
  
  lastLeftValue = lateralSensorsValues[0];
  lastRightValue = lateralSensorsValues[1];
  /*
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
  */
}
