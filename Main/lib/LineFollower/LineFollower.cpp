#include "LineFollower.hpp"
#include "../Logger/Logger.hpp"
#include <Arduino.h>

LineFollower::LineFollower(InputSource *pInputSource, SystemController *pSystemController, MotorController *pMotorController, unsigned char statusPin)
{
  setInputSource(pInputSource);
  setSystemController(pSystemController);
  setMotorController(pMotorController);
  setStatusPin(statusPin);
}

LineFollower::~LineFollower()
{
  // TODO - Como liberar a memória corretamente?
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
  float input = m_pInputSource->getInput();
  float pidOutput = m_pSystemController->run(input);
  if(abs(input) < 0.1)
  {
    m_pMotorController->move(200, 0);
    return;
  }
  /*
  Serial.print("Input: ");
  Serial.println(input);
  Serial.print("PID: ");
  Serial.println(pidOutput);
  */
#ifdef DEBUG
  CurrentLogger->writeLine("Input: %f. PID: %f", input, pidOutput);
#endif

  m_pMotorController->move(200, pidOutput);
  //Serial.println("");
}

bool LineFollower::shouldStop(unsigned long maxTime)
{
  if(millis() - m_StartTime > maxTime)
  {
    Serial.println("true");
    return true;
  }
  return false;
}
