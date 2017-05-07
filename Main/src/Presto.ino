#include "../lib/LineFollower/LineFollower.hpp"
#include "../lib/PIDController/PIDController.hpp"
#include "PrestoMotorController.hpp"
#include "PrestoSensoring.hpp"
#include "Constants.h"
#include "Pins.h"

volatile bool shouldStop;
LineFollower presto;
PrestoSensoring sensoring;
PIDController pidController;
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

  pidController.setSetPoint(0);
  pidController.setSampleTime(10);
  pidController.setOutputLimits(0, 255); // TODO - PWM?
  pidController.setControllerDirection(SystemControllerDirection::Direct);
  pidController.setTunings(12, 40, 0.8);

  // TODO - Descomentar essa linha e colocar os pinos certos do motor
  //motorController.setPins(L_MOTOR_1_PIN, L_MOTOR_2_PIN, R_MOTOR_1_PIN, R_MOTOR_2_PIN);
  motorController.setWheelsRadius(WHEELS_RADIUS);
  motorController.setWheelsDistance(WHEELS_DISTANCE);
  motorController.setVelocities(100, 60); // TODO - Porcentagens?

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
