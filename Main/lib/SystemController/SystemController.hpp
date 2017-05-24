#ifndef SystemController_hpp
#define SystemController_hpp

#include <Arduino.h>
#include "../CompilerDefinitions.h"
#include "../Logger/Logger.hpp"

enum SystemControllerDirection
{
  Inverse = -1,
  Direct = 1
};

class SystemController
{
public:
  SystemController() : m_LastRunTime(millis()) {}
  virtual float run(float intput) = 0;

  virtual void setSampleTime(unsigned long newSampleTime);
  virtual void setSetPoint(float newSetPoint) { m_SetPoint = newSetPoint ;}
  virtual void setOutputLimits(float min, float max);
  virtual void setControllerDirection(SystemControllerDirection direction) { m_ControllerDirection = direction; }

protected:
  unsigned long m_SampleTime;
  float m_SetPoint;
  float m_MinOutput;
  float m_MaxOutput;
	unsigned long m_Now;
  unsigned long m_LastRunTime;

  SystemControllerDirection m_ControllerDirection;

  bool checkTime(unsigned long *pDeltaTime);
};

#endif // SystemController_hpp
