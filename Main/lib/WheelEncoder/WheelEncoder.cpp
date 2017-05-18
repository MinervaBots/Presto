#include "WheelEncoder.hpp"
#include <Arduino.h>

namespace WheelEncoderPrivate
{
  static WheelEncoder *m_pWheelEncoderInstance;
}

WheelEncoder::WheelEncoder(char leftTickPin, char rightTickPin, float wheelRadius, unsigned int ticksPerRevolution)
{
  WheelEncoderPrivate::m_pWheelEncoderInstance = this;
  setWheelRadius(wheelRadius);
  setTicksPerRevolution(ticksPerRevolution);
  pinMode(leftTickPin, INPUT);
  pinMode(rightTickPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(leftTickPin), leftTick, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightTickPin), rightTick, CHANGE);
}

void WheelEncoder::leftTick()
{
  WheelEncoderPrivate::m_pWheelEncoderInstance->m_DeltaLeftTickCount++;
}

void WheelEncoder::rightTick()
{
  WheelEncoderPrivate::m_pWheelEncoderInstance->m_DeltaRightTickCount++;
}

void WheelEncoder::update()
{
  m_DeltaDistanceLeft = 2 * PI * m_WheelRadius * (m_DeltaLeftTickCount / m_TicksPerRevolution);
  m_DeltaDistanceRight = 2 * PI * m_WheelRadius * (m_DeltaRightTickCount / m_TicksPerRevolution);
  m_TotalDistanceLeft += m_DeltaDistanceLeft;
  m_TotalDistanceRight += m_DeltaDistanceRight;
  m_DeltaLeftTickCount = 0;
  m_DeltaRightTickCount = 0;
}
