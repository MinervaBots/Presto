#include "LineFollower.h"
#include <Arduino.h>

LineFollower::LineFollower()
{

}

LineFollower::LineFollower(InputSource *pInputSource, SystemController *pSystemController, MotorController *pMotorController) :
  _pInputSource(pInputSource),
  _pSystemController(pSystemController),
  _pMotorController(pMotorController)
{

}

LineFollower::~LineFollower()
{
  // TODO - Como liberar a memória corretamente?
  /*
  delete _pInputSource;
  delete _pSystemController;
  delete _pMotorController;
  */
}

void LineFollower::start()
{
  _startTime = millis();
}

void LineFollower::stop()
{
  _stopTime = millis();
}


void LineFollower::update()
{
  float pidOutput = _pSystemController->run(_pInputSource->getInput());
  _pMotorController->move(100, pidOutput);
}
