#include "PrestoSensoring.hpp"
#include "../lib/Logger/Logger.hpp"

PrestoSensoring::PrestoSensoring()
{

}

PrestoSensoring::PrestoSensoring(QTRSensorsRC qtrArray, QTRSensorsRC qtrLeft, QTRSensorsRC qtrRight,
  unsigned long leftSampleTime, unsigned long rightSampleTime)
{
  setSensorArray(qtrArray);
  setSensorLeft(qtrLeft);
  setSensorRight(qtrRight);
  setSampleTimes(leftSampleTime, rightSampleTime);
}


void PrestoSensoring::setSensorArray(QTRSensorsRC qtrArray)
{
  m_QtrArray = qtrArray;
  m_SensorWeights = new unsigned int[m_QtrArray.getNumSensors()];
  m_CenterPosition = ((m_QtrArray.getNumSensors() - 1) * 1000) / 2;
}

float PrestoSensoring::getInput()
{
  return (m_QtrArray.readLine(m_SensorWeights, QTR_EMITTERS_ON, true) - m_CenterPosition) / m_CenterPosition;
}

void PrestoSensoring::setSampleTimes(unsigned long leftSampleTime, unsigned long rightSampleTime)
{
  m_LeftSampleTime = leftSampleTime;
  m_RightSampleTime = rightSampleTime;
}

void PrestoSensoring::calibrate(Button commandButton, unsigned char statusLedPin)
{
  while(!commandButton.isPressed());

#ifdef DEBUG
  Logger::CurrentLogger->WriteLine("Iniciando calibração");
#endif

  digitalWrite(statusLedPin, HIGH);

  while(!commandButton.isPressed())
  {
    m_QtrArray.calibrate();
    m_QtrRight.calibrate();
    m_QtrLeft.calibrate();
  }

  m_LeftSensorThreshold = (m_QtrLeft.getCalibratedMinimum(false)[0] + m_QtrLeft.getCalibratedMaximum(false)[0]) / 2;
  m_RightSensorThreshold = (m_QtrRight.getCalibratedMinimum(false)[0] + m_QtrRight.getCalibratedMaximum(false)[0]) / 2;

#ifdef DEBUG
  Logger::CurrentLogger->WriteLine("Calibração concluída");
#endif

  digitalWrite(statusLedPin, HIGH);
  delay(500);
  digitalWrite(statusLedPin, LOW);

  // Espera apertar de novo pra começar o loop
  while(!commandButton.isPressed());
  digitalWrite(statusLedPin, HIGH);
  delay(500);
  digitalWrite(statusLedPin, LOW);
}

void PrestoSensoring::update()
{
  bool leftMark, rightMark;

  unsigned long now = millis();
  unsigned int value = 0;
  if((now - m_LastRun) > m_LeftSampleTime)
  {
    m_QtrLeft.readCalibrated(&value);
    leftMark = value < m_LeftSensorThreshold;
  }

  if((now - m_LastRun) > m_RightSampleTime)
  {
    value = 0;
    m_QtrRight.readCalibrated(&value);
    rightMark = value < m_RightSensorThreshold;
  }
  m_LastRun = now;

  if(leftMark && rightMark) // Interseção da linha na pista
  {
    // Não faz nada :P
  }
  else if(!leftMark && rightMark) // Marca de inicio de prova
  {
    m_RightCount++;
  }
  else if(leftMark && !rightMark) // Marca de inicio/fim de curva
  {
    m_InCurve = !m_InCurve;
  }
}
