#include "PrestoMotorController.hpp"
#include "../lib/MathHelper/MathHelper.h"
#include <Arduino.h>

PrestoMotorController::PrestoMotorController(unsigned char leftPwmPin, unsigned char leftDirectionPin,
  unsigned char rightPwmPin, unsigned char rightDirectionPin) :
  DifferentialDriveController(0, 0, nullptr, nullptr)
{
  setPins(leftPwmPin, leftDirectionPin, rightPwmPin, rightDirectionPin);

  // Velocidade máxima não parece ser uma boa ideia
  setMaxPWM(200);
}

void PrestoMotorController::setPins(unsigned char leftPwmPin, unsigned char leftDirectionPin,
  unsigned char rightPwmPin, unsigned char rightDirectionPin)
{
  m_LeftPwmPin = leftPwmPin;
  m_LeftDirectionPin = leftDirectionPin;

  m_RightPwmPin = rightPwmPin;
  m_RightDirectionPin = rightDirectionPin;

  pinMode(m_LeftPwmPin, OUTPUT);
  pinMode(m_LeftDirectionPin, OUTPUT);

  pinMode(m_RightPwmPin, OUTPUT);
  pinMode(m_RightDirectionPin, OUTPUT);
}

void PrestoMotorController::stop()
{
  DifferentialDriveController::stop();
  analogWrite(m_LeftPwmPin, 0);
  analogWrite(m_RightPwmPin, 0);
}

void PrestoMotorController::update()
{

}

void PrestoMotorController::setMaxPWM(unsigned int maxPWM)
{
  m_MaxPWM = maxPWM;
  if(m_MaxPWM > 255)
  {
    m_MaxPWM = 255;
  }
}

void PrestoMotorController::move(float linearVelocity, float angularVelocity)
{
  linearVelocity *= m_LinearVelocityRatio;

  m_LeftVelocity = linearVelocity + angularVelocity;
  m_RightVelocity = linearVelocity - angularVelocity;

  /*
  Essas linhas de código normalizam os valores de velocidade, resultando em
  um deles sendo igual a 1, e o outro menor que 1.
  Com isso eu garanto que nunca vou ter valores maior que 1 e posso multiplicar
  os valores diretamente pela PWM máxima.
  */
  float maxVelocity = linearVelocity + abs(angularVelocity);
  float normalizedLeftVelocity = abs(m_LeftVelocity) / maxVelocity;
  float normalizedRightVelocity = abs(m_RightVelocity) / maxVelocity;


	analogWrite(m_LeftPwmPin, floor(m_MaxPWM * normalizedLeftVelocity));
	analogWrite(m_RightPwmPin, floor(m_MaxPWM * normalizedRightVelocity));

  digitalWrite(m_LeftDirectionPin, (m_LeftVelocity > 0) ? HIGH : LOW);
  digitalWrite(m_RightDirectionPin, (m_RightVelocity > 0) ? HIGH : LOW);
}
