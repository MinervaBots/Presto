#include "SystemController.hpp"

void SystemController::setSampleTime(unsigned long newSampleTime)
{
  m_SampleTime = newSampleTime;
  /*
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
  */
}

void SystemController::setOutputLimits(float min, float max)
{
	if (min > max)
	{
#ifdef DEBUG
    CurrentLogger->WriteLine("[SystemController::setOutputLimits]: min não pode ser maior que max");
#endif
	}
	m_MinOutput = min;
	m_MaxOutput = max;
}


bool SystemController::checkTime(unsigned long *pDeltaTime)
{
	m_Now = millis();
	*pDeltaTime = (m_Now - m_LastRunTime);
	return *pDeltaTime > m_SampleTime;
}
