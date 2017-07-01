#include "SystemController.hpp"

void SystemController::setSampleTime(unsigned long newSampleTime)
{
  m_SampleTime = newSampleTime;
}

void SystemController::setOutputLimits(float min, float max)
{
	if (min > max)
	{
#ifdef DEBUG
    Serial.println("[SystemController::setOutputLimits]: min não pode ser maior que max");
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
