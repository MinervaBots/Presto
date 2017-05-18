#include "LineFollower.hpp"
#include <Arduino.h>

LineFollower::LineFollower(InputSource *pInputSource, SystemController *pSystemController, MotorController *pMotorController)
{
  setInputSource(pInputSource);
  setSystemController(pSystemController);
  setMotorController(pMotorController);
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
  m_IsRunning = true;
  m_StartTime = millis();
}

void LineFollower::stop()
{
  m_StopTime = millis();
  m_pMotorController->stop();
  m_IsRunning = false;
}

void LineFollower::update()
{
  float pidOutput = m_pSystemController->run(m_pInputSource->getInput());
  m_pMotorController->move(70, pidOutput);
}
