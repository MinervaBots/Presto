#include "../lib/LineFollower/LineFollower.h"
#include "../lib/PIDController/PIDController.h"
#include "PrestoMotorController.h"
#include "PrestoSensoring.h"
#include "Constants.h"
#include "Pins.h"

float input;
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

  motorController.setPins(L_MOTOR_1_PIN, L_MOTOR_2_PIN, R_MOTOR_1_PIN, R_MOTOR_2_PIN);
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
  if(sensoring.shouldStop())
  {
    presto.stop();
    return;
  }

  motorController.inCurve = sensoring.inCurve();
  presto.update();
}

void killSwitch()
{
  presto.stop();
}
