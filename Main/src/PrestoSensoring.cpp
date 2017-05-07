#include "PrestoSensoring.hpp"

PrestoSensoring::PrestoSensoring()
{

}

PrestoSensoring::PrestoSensoring(QTRSensorsRC qtrArray, QTRSensorsRC qtrLeft, QTRSensorsRC qtrRight,
  unsigned long leftSampleTime, unsigned long rightSampleTime, unsigned int *sensorWeights, LowPassFilter *pLowPassFilter)
{
  setSensorArray(qtrArray);
  setSensorLeft(qtrLeft);
  setSensorRight(qtrRight);
  setSampleTimes(leftSampleTime, rightSampleTime);
  setSensorWeights(sensorWeights);
  setLowPassFilter(pLowPassFilter);
}


void PrestoSensoring::setSensorArray(QTRSensorsRC qtrArray)
{
  m_QtrArray = qtrArray;
  m_CenterPosition = ((m_QtrArray.getNumSensors() - 1) * 1000) / 2;
}

float PrestoSensoring::getInput()
{
  float sensorOutput = (m_QtrArray.readLine(m_SensorWeights, QTR_EMITTERS_ON, true) - m_CenterPosition) / m_CenterPosition;

  if(m_pLowPassFilter == nullptr)
    return sensorOutput;

  return m_pLowPassFilter->run(sensorOutput);
  //return erro_maximo * ((m_QtrArray.readLine(m_SensorWeights, QTR_EMITTERS_ON, WHITE_LINE) - CENTER_POSITION)/CENTER_POSITION);
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
  bool leftMark, rightMark;

  unsigned long now = millis();
  unsigned int value = 0;
  if((now - m_LastRun) > m_LeftSampleTime)
  {
    m_QtrLeft.readCalibrated(&value);
    leftMark = value < 100;
  }

  if((now - m_LastRun) > m_RightSampleTime)
  {
    value = 0;
    m_QtrRight.readCalibrated(&value);
    rightMark = value < 100;
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
