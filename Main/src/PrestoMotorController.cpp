#include "PrestoMotorController.hpp"
#include "../lib/MathHelper/MathHelper.h"
#include <Arduino.h>

PrestoMotorController::PrestoMotorController()
{

}

PrestoMotorController::PrestoMotorController(unsigned int leftInPin1, unsigned int leftInPin2,
  unsigned int rightInPin1, unsigned int rightInPin2,
  float wheelsRadius, float wheelsDistance, WheelEncoder *pWheelEncoder) :
  DifferentialDriveController(wheelsRadius, wheelsDistance, nullptr, pWheelEncoder)
{
  setPins(leftInPin1, leftInPin2, rightInPin1, rightInPin2);
}

void PrestoMotorController::setPins(unsigned int leftInPin1, unsigned int leftInPin2,
  unsigned int rightInPin1, unsigned int rightInPin2)
{
  m_LeftInPin1 = leftInPin1;
  m_LeftInPin2 = leftInPin2;

  m_RightInPin1 = rightInPin1;
  m_RightInPin2 = rightInPin2;

  pinMode(m_LeftInPin1, OUTPUT);
  pinMode(m_LeftInPin2, OUTPUT);

  pinMode(m_RightInPin1, OUTPUT);
  pinMode(m_RightInPin2, OUTPUT);
}

void PrestoMotorController::stop()
{
  DifferentialDriveController::stop();
  analogWrite(m_LeftInPin1, 255);
  analogWrite(m_LeftInPin2, 255);
  analogWrite(m_RightInPin1, 255);
  analogWrite(m_RightInPin2, 255);
}

void PrestoMotorController::calculatePwm()
{
  m_LeftPwm = 255 * abs(getLeftVelocity()) / (KM * TENSAO_DE_ALIMENTACAO);
  if(m_LeftPwm > 255)
  {
      m_LeftPwm = 255;
  }

  m_RightPwm = 255 * abs(getRightVelocity()) / (KM * TENSAO_DE_ALIMENTACAO);
  if(m_RightPwm > 255)
  {
      m_RightPwm = 255;
  }
}

void PrestoMotorController::move(float linearVelocity, float angularVelocity)
{
  DifferentialDriveController::move(linearVelocity, angularVelocity);

  calculatePwm();

	if(getLeftVelocity() > 0)
	{
		analogWrite(m_LeftInPin1, m_LeftPwm);
		analogWrite(m_LeftInPin2, 0);
	}
	else
	{
		analogWrite(m_LeftInPin2, m_LeftPwm);
		analogWrite(m_LeftInPin1, 0);
	}

  if(getRightVelocity() > 0)
	{
		analogWrite(m_RightInPin1, m_RightPwm);
		analogWrite(m_RightInPin2, 0);
	}
	else
	{
		analogWrite(m_RightInPin2, m_RightPwm);
		analogWrite(m_RightInPin1, 0);
	}
}
