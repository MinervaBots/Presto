#include "LineFollower.hpp"
#include "../Logger/Logger.hpp"
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
  float input = m_pInputSource->getInput();
  float pidOutput = m_pSystemController->run(input);

#ifdef DEBUG
  CurrentLogger->WriteLine("Input: %f. PID: %f", input, pidOutput);
#endif

  m_pMotorController->move(70, pidOutput);
}
