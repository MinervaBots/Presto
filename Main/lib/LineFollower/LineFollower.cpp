#include <Arduino.h>
#include "LineFollower.hpp"

LineFollower::LineFollower(InputSource *pInputSource, SystemController *pSystemController, MotorController *pMotorController, unsigned char statusPin)
{
  setInputSource(pInputSource);
  setSystemController(pSystemController);
  setMotorController(pMotorController);
  setStatusPin(statusPin);
  setLinearVelocity(75);
}

LineFollower::~LineFollower()
{
  /*
  delete m_pInputSource;
  delete m_pSystemController;
  delete m_pMotorController;
  */
}

void LineFollower::start()
{
  digitalWrite(m_StatusPin, HIGH);
  m_IsRunning = true;
  m_StartTime = millis();
}

void LineFollower::stop()
{
  digitalWrite(m_StatusPin, LOW);
  m_StopTime = millis();
  m_pMotorController->stop();
  m_IsRunning = false;
}

void LineFollower::setStatusPin(unsigned char statusPin)
{
  m_StatusPin = statusPin;
  pinMode(m_StatusPin, OUTPUT);
}

void LineFollower::update()
{
  float lineError = m_pInputSource->getInput();
  float pidOutput = m_pSystemController->run(lineError);

/*
  Serial.print("Line Error: ");
  Serial.print(lineError);
  Serial.print("\t");
  Serial.print("PID: ");
  Serial.println(pidOutput);
  */
  /*
  Se o erro for muito pequeno, não vale a pena tentar compensar com PID.
  Seria necessário ter as constantes perfeitamente ajustadas, caso contrario,
  ele supercompensaria e o erro só trocaria de lado, fazendo com que ele oscile
  desnecessáriamente.
  A solução que eu empreguei amenizar esse problema, basicamente aumentando o
  intervalo de tempo entre a ocorrencia dele, já que o erro terá que ser um
  pouquinho maior para afetar o sistema.

  OBS: Não tenho certeza, mas acho que podemos mover a avaliação do PID pra
  baixo desse 'if'. Isso vai evitar que façamos calculos inutilmente, mas também
  poderia causar complicações com o controle I e D, que dependem da passagem do
  tempo.
  */
  /*
  if(abs(input) < 0.04)
  {
    m_pMotorController->move(m_LinearVelocity*1.5, 0);
    return;
  }
  //*/

  float linearVelocityCorrection = 0.5 + (1 / (1 + exp(abs(lineError * 7.635))));
  float linearVelocity = m_LinearVelocity * linearVelocityCorrection;
  m_pMotorController->move(linearVelocity, pidOutput);
  //delay(200);
}

bool LineFollower::shouldStop(unsigned long maxTime)
{
  return (millis() - m_StartTime > maxTime);
}
