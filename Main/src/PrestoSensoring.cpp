#include "PrestoSensoring.h"
#include "Constants.h"

PrestoSensoring::PrestoSensoring()
{

}

PrestoSensoring::PrestoSensoring(QTRSensorsRC qtrArray, QTRSensorsRC qtrLeft, QTRSensorsRC qtrRight,
  unsigned long leftSampleTime, unsigned long rightSampleTime, unsigned int *sensorWeights)
{
  setSensorArray(qtrArray);
  setSensorLeft(qtrLeft);
  setSensorRight(qtrRight);
  setSampleTimes(leftSampleTime, rightSampleTime);
  setSensorWeights(sensorWeights);
}

float PrestoSensoring::getInput()
{
  return erro_maximo*((_qtrArray.readLine(_sensorWeights, QTR_EMITTERS_ON, WHITE_LINE) - CENTER_POSITION)/CENTER_POSITION);
}

bool PrestoSensoring::shouldStop()
{
  return _rightCount >= 2;
}

bool PrestoSensoring::inCurve()
{
  return _inCurve;
}

void PrestoSensoring::setSensorArray(QTRSensorsRC qtrArray)
{
  _qtrArray = qtrArray;
}

void PrestoSensoring::setSensorLeft(QTRSensorsRC qtrLeft)
{
  _qtrLeft = qtrLeft;
}

void PrestoSensoring::setSensorRight(QTRSensorsRC qtrRight)
{
  _qtrRight = qtrRight;
}
void PrestoSensoring::setSampleTimes(unsigned long leftSampleTime, unsigned long rightSampleTime)
{
  _leftSampleTime = leftSampleTime;
  _rightSampleTime = rightSampleTime;
}

void PrestoSensoring::setSensorWeights(unsigned int *sensorWeights)
{
  _sensorWeights = sensorWeights;
}

void PrestoSensoring::calibrate(Button commandButton, unsigned int statusLedPin)
{
  while(!commandButton.isPressed());

#ifdef DEBUG
  Serial.println("Iniciando calibração");
#endif

  digitalWrite(statusLedPin, HIGH);

  while(!commandButton.isPressed())
  {
    _qtrArray.calibrate();
    _qtrRight.calibrate();
    _qtrLeft.calibrate();
  }

#ifdef DEBUG
  Serial.println("Calibração concluída");
#endif
}

void PrestoSensoring::update()
{
  unsigned long now = millis();
  unsigned int value = 0;
  if((now - _lastRun) > _leftSampleTime)
  {
    _qtrLeft.readCalibrated(&value);
    if (value < 100)
    {
      _inCurve = !_inCurve;
    }
  }

  if((now - _lastRun) > _rightSampleTime)
  {
    value = 0;
    _qtrRight.readCalibrated(&value);
    if (value < 100)
    {
      _rightCount++;
    }
  }
  _lastRun = now;
}
