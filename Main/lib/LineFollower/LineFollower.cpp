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

#ifdef DEBUG
  Serial.print("Line Error: ");
  Serial.print(lineError);
  Serial.print("\t");
  Serial.print("PID: ");
  Serial.println(pidOutput);
  delay(250);
#endif

  //float linearVelocityCorrection = 0.5 + (1 / (1 + exp(abs(lineError * 7.635))));
  float linearVelocity = m_LinearVelocity;// * linearVelocityCorrection;
  m_pMotorController->move(linearVelocity, pidOutput);
}

bool LineFollower::shouldStop(unsigned long maxTime)
{
  return (millis() - m_StartTime > maxTime);
}
