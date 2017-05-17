#include "../lib/LineFollower/LineFollower.hpp"

#ifdef USE_NON_LINEAR_PID

#include "../lib/PIDController/NonLinearPIDController.hpp"

#else

#include "../lib/PIDController/PIDController.hpp"

#endif

#include "PrestoMotorController.hpp"
#include "PrestoSensoring.hpp"
#include "Pins.h"
#include "../lib/Filter/SimpleMovingAverageFilter.hpp"

volatile bool shouldStop;
LineFollower presto;
PrestoSensoring sensoring;
SimpleMovingAverageFilter simpleMovingAverageFilter(5);

#ifdef USE_NON_LINEAR_PID

NonLinearPIDController pidController;

#else

PIDController pidController;

#endif

PrestoMotorController motorController;

unsigned int sensorWeights[sizeof(SensorArrayPins) / sizeof(char)];

void setup()
{
#ifdef DEBUG
  Serial.begin(9600);
#endif
  attachInterrupt(digitalPinToInterrupt(KILLSWITCH_PIN), killSwitch, HIGH);

  sensoring.setSensorArray(QTRSensorsRC(SensorArrayPins, sizeof(SensorArrayPins) / sizeof(char), 200));
  sensoring.setSensorLeft(QTRSensorsRC(SensorLeftBorderPins, 1, 3000));
  sensoring.setSensorRight(QTRSensorsRC(SensorRightBorderPins, 1, 3000));
  sensoring.setSampleTimes(120, 150);
  sensoring.setSensorWeights(sensorWeights);
  sensoring.setFilter(&simpleMovingAverageFilter);

  pidController.setSetPoint(0);
  pidController.setSampleTime(10);
  pidController.setOutputLimits(-100, 100);
  pidController.setControllerDirection(SystemControllerDirection::Direct);
  pidController.setTunings(12, 40, 0.8);

#ifdef USE_NON_LINEAR_PID
  pidController.setMaxError(10.0);
  pidController.setNonLinearConstanteCoeficients(0.5, 2, 0.5, 2, 2); // Pra ficar de acordo com os ultimos valores na antiga versão
#endif


  motorController.setPins(L_MOTOR_1_PIN, L_MOTOR_2_PIN, R_MOTOR_1_PIN, R_MOTOR_2_PIN);
  motorController.setWheelsRadius(0.185);
  motorController.setWheelsDistance(0.143); // TODO - Medir isso novamente. Por causa do encoder, as rodas podem ficar mais proximas

  presto.setSystemController(&pidController);
  presto.setMotorController(&motorController);

  sensoring.calibrate(Button(COMMAND_BUTTON_PIN, PULLDOWN), STATUS_LED_PIN);

  presto.setInputSource(&sensoring);
  presto.start();
}


void loop()
{
  sensoring.update();
  if(sensoring.shouldStop() || shouldStop)
  {
    presto.stop();
    return;
  }
  motorController.inCurve = sensoring.inCurve();
  presto.update();
}

void killSwitch()
{
  shouldStop = true;
}
