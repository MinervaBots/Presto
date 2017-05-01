#include "PrestoSensoring.hpp"
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
  return erro_maximo*((m_QtrArray.readLine(m_SensorWeights, QTR_EMITTERS_ON, WHITE_LINE) - CENTER_POSITION)/CENTER_POSITION);
}

void PrestoSensoring::setSampleTimes(unsigned long leftSampleTime, unsigned long rightSampleTime)
{
  m_LeftSampleTime = leftSampleTime;
  m_RightSampleTime = rightSampleTime;
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
    m_QtrArray.calibrate();
    m_QtrRight.calibrate();
    m_QtrLeft.calibrate();
  }

#ifdef DEBUG
  Serial.println("Calibração concluída");
#endif
}

void PrestoSensoring::update()
{
  unsigned long now = millis();
  unsigned int value = 0;
  if((now - m_LastRun) > m_LeftSampleTime)
  {
    m_QtrLeft.readCalibrated(&value);
    if (value < 100)
    {
      m_InCurve = !m_InCurve;
    }
  }

  if((now - m_LastRun) > m_RightSampleTime)
  {
    value = 0;
    m_QtrRight.readCalibrated(&value);
    if (value < 100)
    {
      m_RightCount++;
    }
  }
  m_LastRun = now;
}
