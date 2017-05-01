#include "PrestoMotorController.hpp"
#include "../lib/MathHelper/MathHelper.h"
#include <Arduino.h>

PrestoMotorController::PrestoMotorController()
{

}

PrestoMotorController::PrestoMotorController(int leftPin1, int leftPin2, int rightPin1, int rightPin2,
  float maxVelocity, float inCurveVelocity,
  float wheelsRadius, float wheelsDistance, WheelEncoder *pWheelEncoder) :
  DifferentialDriveController(nullptr, wheelsRadius, wheelsDistance, pWheelEncoder)
{
  setPins(leftPin1, leftPin2, rightPin1, rightPin2);
  setVelocities(maxVelocity, inCurveVelocity);
}

void PrestoMotorController::setPins(int leftPin1, int leftPin2, int rightPin1, int rightPin2)
{
  pinMode(leftPin1, OUTPUT);
  pinMode(leftPin2, OUTPUT);
  pinMode(rightPin1, OUTPUT);
  pinMode(rightPin2, OUTPUT);
}

void PrestoMotorController::setVelocities(float maxVelocity, float inCurveVelocity)
{
  m_MaxVelocity = maxVelocity;
  m_InCurveVelocity = inCurveVelocity;
}

void PrestoMotorController::move(float linearVelocity, float angularVelocity)
{
  if(inCurve)
  {
    linearVelocity = clamp(linearVelocity, 0, m_InCurveVelocity);
  }
  else
  {
    linearVelocity = clamp(linearVelocity, 0, m_MaxVelocity);
  }
  DifferentialDriveController::move(linearVelocity, angularVelocity);

  // TODO - PWM dos motores
  /*
  if(angularVelocity > 0)
  {
    analogWrite(L_MOTOR_1, SPEED);
    analogWrite(L_MOTOR_2, 0);
    if(SPEED - pid_output < 0)
    {
      analogWrite(R_MOTOR_1, 0);
      analogWrite(R_MOTOR_2, abs(floor(SPEED - pid_output)));
    }
    else
    {
      analogWrite(R_MOTOR_1,floor(SPEED - pid_output));
      analogWrite(R_MOTOR_2,0);
    }
  }
  else if(angularVelocity < 0)
  {
    analogWrite(R_MOTOR_1, SPEED);
    analogWrite(R_MOTOR_2, 0);
    if(SPEED + pid_output < 0)
    {
      analogWrite(L_MOTOR_1, 0);
      analogWrite(L_MOTOR_2, floor(SPEED + pid_output));
    }
    else
    {
      analogWrite(L_MOTOR_1, floor(SPEED + pid_output));
      analogWrite(L_MOTOR_2, 0);
    }
  }
  else
  {

  }
  */
}
