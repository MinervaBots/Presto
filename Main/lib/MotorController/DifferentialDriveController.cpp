#include "DifferentialDriveController.hpp"
#include <math.h>

DifferentialDriveController::DifferentialDriveController()
{
  m_pPosition = nullptr;
}

DifferentialDriveController::DifferentialDriveController(Position* pPosition, float wheelsRadius, float wheelsDistance, WheelEncoder *pWheelEncoder)
{
  setWheelsRadius(wheelsRadius);
  setWheelsDistance(wheelsDistance);
  setEncoder(pWheelEncoder);
  setPositionPointer(pPosition);
}

void DifferentialDriveController::update()
{
  m_pWheelEncoder->update();
}

void DifferentialDriveController::move(float linearVelocity, float angularVelocity)
{
  m_LeftVelocity = ((2 * linearVelocity) + (angularVelocity * m_WheelsDistance)) / (2 * m_WheelsRadius);
  m_RightVelocity = ((2 * linearVelocity) - (angularVelocity * m_WheelsDistance)) / (2 * m_WheelsRadius);

  if(!m_pPosition)
    return;

  // Baseado no modelo
  /*
  float headingPrime += (m_WheelsRadius / m_WheelsDistance) * (m_RightVelocity - m_LeftVelocity);
  */

  // Usando o encoder
  float deltaEncoder = m_pWheelEncoder->getDeltaDistance();
  float deltaEncoderLeft = m_pWheelEncoder->getDeltaDistanceLeft();
  float deltaEncoderRight = m_pWheelEncoder->getDeltaDistanceRight();

  float xPrime = deltaEncoder * cos(m_pPosition->getHeading());
  float yPrime = deltaEncoder * sin(m_pPosition->getHeading());
  float headingPrime = (deltaEncoderRight - deltaEncoderLeft) / m_WheelsDistance;


  m_pPosition->setX(m_pPosition->getX() + xPrime);
  m_pPosition->setY(m_pPosition->getY() + yPrime);
  m_pPosition->setHeading(m_pPosition->getHeading() + headingPrime);
}
