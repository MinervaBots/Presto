#include "PrestoMotorController.hpp"
#include "../lib/MathHelper/MathHelper.h"
#include <Arduino.h>

PrestoMotorController::PrestoMotorController()
{

}

PrestoMotorController::PrestoMotorController(unsigned int leftPwmPin, unsigned int leftInPin1, unsigned int leftInPin2,
  unsigned int rightPwmPin, unsigned int rightInPin1, unsigned int rightInPin2, unsigned int standbyPin,
  float maxVelocity, float inCurveVelocity,
  float wheelsRadius, float wheelsDistance, WheelEncoder *pWheelEncoder) :
  DifferentialDriveController(wheelsRadius, wheelsDistance, nullptr, pWheelEncoder)
{
  setPins(leftPwmPin, leftInPin1, leftInPin2, rightPwmPin, rightInPin1, rightInPin2, standbyPin);
  setVelocities(maxVelocity, inCurveVelocity);
}

void PrestoMotorController::setPins(unsigned int leftPwmPin, unsigned int leftInPin1, unsigned int leftInPin2,
  unsigned int rightPwmPin, unsigned int rightInPin1, unsigned int rightInPin2, unsigned int standbyPin)
{
  m_LeftPwmPin = leftPwmPin;
  m_LeftInPin1 = leftInPin1;
  m_LeftInPin2 = leftInPin2;

  m_RightPwmPin = rightPwmPin;
  m_RightInPin1 = rightInPin1;
  m_RightInPin2 = rightInPin2;

  m_StandbyPin = standbyPin;

  pinMode(m_LeftPwmPin, OUTPUT);
  pinMode(m_LeftInPin1, OUTPUT);
  pinMode(m_LeftInPin2, OUTPUT);

  pinMode(m_RightPwmPin, OUTPUT);
  pinMode(m_RightInPin1, OUTPUT);
  pinMode(m_RightInPin2, OUTPUT);

  pinMode(m_StandbyPin, OUTPUT);
}

void PrestoMotorController::setVelocities(float maxVelocity, float inCurveVelocity)
{
  m_MaxVelocity = maxVelocity;
  m_InCurveVelocity = inCurveVelocity;
}

void PrestoMotorController::stop()
{
  digitalWrite(m_StandbyPin, LOW);
  DifferentialDriveController::stop();
}

void PrestoMotorController::move(float linearVelocity, float angularVelocity)
{
  // TODO - Limitar a velocidade angular também?
  if(inCurve)
  {
    linearVelocity = clamp(linearVelocity, 0, m_InCurveVelocity);
  }
  else
  {
    linearVelocity = clamp(linearVelocity, 0, m_MaxVelocity);
  }
  DifferentialDriveController::move(linearVelocity, angularVelocity);

  digitalWrite(m_StandbyPin, HIGH);

  // Todo checar essa constante de divisão
  int leftPWM = round(abs(lerp(0, 255, getLeftVelocity() / 100)));
  analogWrite(m_LeftPwmPin, leftPWM);
  if(getLeftVelocity() > 0)
  {
    digitalWrite(m_LeftInPin1, HIGH);
    digitalWrite(m_LeftInPin2, LOW);
  }
  else
  {
    digitalWrite(m_LeftInPin1, LOW);
    digitalWrite(m_LeftInPin2, HIGH);
  }

  int rightPWM = round(abs(lerp(0, 255, getRightVelocity() / 100)));
  analogWrite(m_RightPwmPin, rightPWM);
  if(getRightVelocity() > 0)
  {
    digitalWrite(m_RightInPin1, HIGH);
    digitalWrite(m_RightInPin2, LOW);
  }
  else
  {
    digitalWrite(m_RightInPin1, LOW);
    digitalWrite(m_RightInPin2, HIGH);
  }
}
