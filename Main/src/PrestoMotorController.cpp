#include "PrestoMotorController.hpp"
#include "../lib/MathHelper/MathHelper.h"
#include <Arduino.h>

PrestoMotorController::PrestoMotorController(unsigned char leftPwmPin, unsigned char leftDirectionPin,
  unsigned char rightPwmPin, unsigned char rightDirectionPin) :
  DifferentialDriveController(0, 0)
{
  setPins(leftPwmPin, leftDirectionPin, rightPwmPin, rightDirectionPin);
}

void PrestoMotorController::setPins(unsigned char leftPin1, unsigned char leftPin2,
  unsigned char rightPin1, unsigned char rightPin2)
{
  m_LeftPin1 = leftPin1;
  m_LeftPin2 = leftPin2;

  m_RightPin1 = rightPin1;
  m_RightPin2 = rightPin2;

  pinMode(m_LeftPin1, OUTPUT);
  pinMode(m_LeftPin2, OUTPUT);

  pinMode(m_RightPin1, OUTPUT);
  pinMode(m_RightPin2, OUTPUT);
}

void PrestoMotorController::stop()
{
  DifferentialDriveController::stop();
  digitalWrite(m_LeftPin1, LOW);
  digitalWrite(m_LeftPin2, LOW);
  digitalWrite(m_RightPin1, LOW);
  digitalWrite(m_RightPin2, LOW);
}

void PrestoMotorController::update()
{

}

void PrestoMotorController::move(float linearVelocity, float angularVelocity)
{
  linearVelocity = constrain(linearVelocity, -255, 255);

  m_LeftVelocity = constrain(linearVelocity + angularVelocity, -255, 255);
  m_RightVelocity = constrain(linearVelocity - angularVelocity, -255, 255);

  if(angularVelocity == 0)
  {
    analogWrite(m_LeftPin1, 0);
    analogWrite(m_LeftPin2, abs(floor(m_LeftVelocity)));

    analogWrite(m_RightPin1, 0);
    analogWrite(m_RightPin2, abs(floor(m_RightVelocity)));
  }
  else if(angularVelocity < 0)
  {
    analogWrite(m_LeftPin1, abs(floor(m_LeftVelocity)));
    analogWrite(m_LeftPin2, 0);
    m_IsLeftForward = true;

    if(m_RightVelocity < 0)
    {
      analogWrite(m_RightPin1, abs(floor(m_RightVelocity)));
      analogWrite(m_RightPin2, 0);
    }
    else
    {
      analogWrite(m_RightPin1, 0);
      analogWrite(m_RightPin2, abs(floor(m_RightVelocity)));
    }
  }
  else
  {
    analogWrite(m_RightPin1, abs(floor(m_RightVelocity)));
    analogWrite(m_RightPin2, 0);
    if(m_LeftVelocity < 0)
    {
      m_IsLeftForward = false;
      analogWrite(m_LeftPin1, abs(floor(m_LeftVelocity)));
      analogWrite(m_LeftPin2, 0);
    }
    else
    {
      m_IsLeftForward = true;
      analogWrite(m_LeftPin1, 0);
      analogWrite(m_LeftPin2, abs(floor(m_LeftVelocity)));
    }
  }
  Serial.print("Direita: ");
  Serial.println(m_RightVelocity);
  Serial.print("Esquerda: ");
  Serial.println(m_LeftVelocity);
}
