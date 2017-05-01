#include "SystemController.hpp"


void SystemController::setSampleTime(unsigned long newSampleTime)
{
  if(m_SampleTime == 0)
  {
    m_SampleTime = newSampleTime;
  }
  else if (newSampleTime > 0)
	{
		m_SampleTime = newSampleTime;
	}
	else
	{
    #ifdef USE_SERIAL
    Serial.println("[SystemController::setSampleTime]: newSampleTime não pode ser menor ou igual a zero");
    #endif
	}
}

void SystemController::setOutputLimits(float min, float max)
{
	if (min > max)
	{
    #ifdef USE_SERIAL
    Serial.println("[SystemController::setOutputLimits]: min não pode ser maior que max");
    #endif
	}
	m_MinOutput = min;
	m_MaxOutput = max;
}
