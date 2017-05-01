#include "WheelEncoder.h"
#include <Arduino.h>

WheelEncoder::WheelEncoder(int leftTickPin, int rightTickPin, float wheelRadius, int ticksPerRevolution) :
  _wheelRadius(wheelRadius),
  _ticksPerRevolution(ticksPerRevolution)
{
  _pInstance = this;
  pinMode(leftTickPin, INPUT);
  pinMode(rightTickPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(leftTickPin), leftTick, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightTickPin), rightTick, CHANGE);
}

void WheelEncoder::leftTick()
{
  _pInstance->_deltaLeftTickCount++;
  //_pInstance->_deltaDistanceLeft += _pInstance->_tickArc;
  //_pInstance->_totalDistanceLeft += _pInstance->_tickArc;
}

void WheelEncoder::rightTick()
{
  _pInstance->_deltaRightTickCount++;
  //_pInstance->_deltaDistanceRight += _pInstance->_tickArc;
  //_pInstance->_totalDistanceRight += _pInstance->_tickArc;
}

void WheelEncoder::update()
{
  _deltaDistanceLeft = 2 * PI * _wheelRadius * (_deltaLeftTickCount / _ticksPerRevolution);
  _deltaDistanceRight = 2 * PI * _wheelRadius * (_deltaRightTickCount / _ticksPerRevolution);
  _totalDistanceLeft += _deltaDistanceLeft;
  _totalDistanceRight += _deltaDistanceRight;
}

float WheelEncoder::getTotalDistance()
{
  return ((getTotalDistanceRight() + getTotalDistanceLeft()) / 2);
}

float WheelEncoder::getTotalDistanceLeft()
{
  return _totalDistanceLeft;
}

float WheelEncoder::getTotalDistanceRight()
{
  return _totalDistanceRight;
}

float WheelEncoder::getDeltaDistance()
{
  return ((getDeltaDistanceRight() + getDeltaDistanceLeft()) / 2);
}

float WheelEncoder::getDeltaDistanceLeft()
{
  return _deltaDistanceLeft;
}

float WheelEncoder::getDeltaDistanceRight()
{
  return _deltaDistanceRight;
}
