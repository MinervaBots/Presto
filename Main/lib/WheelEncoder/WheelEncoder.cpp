#include "WheelEncoder.hpp"
#include <Arduino.h>

WheelEncoder::WheelEncoder(int leftTickPin, int rightTickPin, float wheelRadius, unsigned int ticksPerRevolution)
{
  m_pInstance = this;
  setWheelRadius(wheelRadius);
  setTicksPerRevolution(ticksPerRevolution);
  pinMode(leftTickPin, INPUT);
  pinMode(rightTickPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(leftTickPin), leftTick, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightTickPin), rightTick, CHANGE);
}

void WheelEncoder::leftTick()
{
  m_pInstance->m_DeltaLeftTickCount++;
}

void WheelEncoder::rightTick()
{
  m_pInstance->m_DeltaRightTickCount++;
}

void WheelEncoder::update()
{
  m_DeltaDistanceLeft = 2 * PI * m_WheelRadius * (m_DeltaLeftTickCount / m_TicksPerRevolution);
  m_DeltaDistanceRight = 2 * PI * m_WheelRadius * (m_DeltaRightTickCount / m_TicksPerRevolution);
  m_TotalDistanceLeft += m_DeltaDistanceLeft;
  m_TotalDistanceRight += m_DeltaDistanceRight;
}
