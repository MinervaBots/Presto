#include "DifferentialDriveController.hpp"
#include "../MathHelper/MathHelper.h"
#include "../Logger/Logger.hpp"

DifferentialDriveController::DifferentialDriveController(float wheelsRadius, float wheelsDistance, Position* pPosition, WheelEncoder *pWheelEncoder)
{
  setWheelsRadius(wheelsRadius);
  setWheelsDistance(wheelsDistance);
  setPositionPointer(pPosition);
  setEncoder(pWheelEncoder);
}


void DifferentialDriveController::setEncoder(WheelEncoder *pWheelEncoder)
{
  m_pWheelEncoder = pWheelEncoder;
  if(m_pWheelEncoder != nullptr)
  {
    m_pWheelEncoder->setWheelRadius(m_WheelsRadius);
  }
}

void DifferentialDriveController::setWheelsRadius(float wheelsRadius)
{
#ifdef DEBUG
  if(wheelsRadius <= 0)
  {
    CurrentLogger->WriteLine("[DifferentialDriveController::setWheelsRadius] O raio das rodas não pode ser menor ou igual a 0");
    return;
  }
#endif
  m_WheelsRadius = wheelsRadius;
  if(m_pWheelEncoder != nullptr)
  {
    m_pWheelEncoder->setWheelRadius(m_WheelsRadius);
  }
}


void DifferentialDriveController::setWheelsDistance(float wheelsDistance)
{
#ifdef DEBUG
  if(wheelsDistance <= 0)
  {
    CurrentLogger->WriteLine("[DifferentialDriveController::setWheelsDistance] A distância entre as rodas deve ser maior ou igual a 0");
    return;
  }
#endif
  m_WheelsDistance = wheelsDistance;
}

void DifferentialDriveController::update()
{
  m_pWheelEncoder->update();
  updatePosition();
}

void DifferentialDriveController::move(float linearVelocity, float angularVelocity)
{
  m_LeftVelocity = ((2 * linearVelocity) + (angularVelocity * m_WheelsDistance)) / (2 * m_WheelsRadius);
  m_RightVelocity = ((2 * linearVelocity) - (angularVelocity * m_WheelsDistance)) / (2 * m_WheelsRadius);
  // Saturação de velocidade é feita nas implementações derivadas que precisarem disso
}

void DifferentialDriveController::updatePosition()
{
  if(!m_pPosition)
    return;

  float xPrime, yPrime, headingPrime;
  if(!m_pWheelEncoder)
  {
    /*
    Não tendo um encoder definido, calcula a variação baseado no modelo de controle.
    Isso tá longe ser ser correto já que essas velocidades viram PWM e não são
    garantem que o motor vai girar necessáriamente nessa velocidade.

    Pra isso se aproximar com a realidade precisaria de um modelo de controle bem mais robusto
    que eu não sou capaz de implementar agora.
    */
    xPrime += (m_WheelsRadius / 2) * (m_RightVelocity + m_LeftVelocity) * cos(m_pPosition->getHeading());
    yPrime += (m_WheelsRadius / 2) * (m_RightVelocity + m_LeftVelocity) * sin(m_pPosition->getHeading());
    headingPrime += (m_WheelsRadius / m_WheelsDistance) * (m_RightVelocity - m_LeftVelocity);
  }
  else
  {
    /*
    Tendo um encoder, vamos usar os dados dele.

    Aqui o resultado é bem mais correto. Uma pista plana como a do seguidor não
    enfrenta problemas de deslizamento, então a exatidão desses dados só depende,
    da resolução e confiabilidade do encoder.
    */
    float deltaEncoder = m_pWheelEncoder->getDeltaDistance();
    float deltaEncoderLeft = m_pWheelEncoder->getDeltaDistanceLeft();
    float deltaEncoderRight = m_pWheelEncoder->getDeltaDistanceRight();

    xPrime = deltaEncoder * cos(m_pPosition->getHeading());
    yPrime = deltaEncoder * sin(m_pPosition->getHeading());
    headingPrime = (deltaEncoderRight - deltaEncoderLeft) / m_WheelsDistance;
  }
  /*
  No bloco acima calculamos apenas a variação da posição (x', y', e heading').

  Pra ter a nova posição precisamos somar aos valores que tinhamos antes desse update.
  */
  m_pPosition->setX(m_pPosition->getX() + xPrime);
  m_pPosition->setY(m_pPosition->getY() + yPrime);
  m_pPosition->setHeading(m_pPosition->getHeading() + headingPrime);
}
