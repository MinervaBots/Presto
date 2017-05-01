#include "SystemController.h"


void SystemController::setSetPoint(float newSetPoint)
{
  _setPoint = newSetPoint;
}

void SystemController::setSampleTime(unsigned long newSampleTime)
{
  if(_sampleTime == 0)
  {
    _sampleTime = newSampleTime;
  }
  else if (newSampleTime > 0)
	{
		_sampleTime = newSampleTime;
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
	_minOutput = min;
	_maxOutput = max;
}

void SystemController::setControllerDirection(SystemControllerDirection direction)
{
	_controllerDirection = direction;
}
