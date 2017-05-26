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
  // Inicializa tudo em zero
  memset(m_SensorWeights, 0, m_QtrArray.getNumSensors() * sizeof(int));

  /*
  Calcula qual o valor central que o array retorna
  Como QTRSensor retorna o número [0..n-1] do sensor * 1000:
  */
  m_CenterPosition = ((m_QtrArray.getNumSensors() - 1) * 1000) / 2;
}

float PrestoSensoring::getInput()
{
  /*
  Normaliza a saída do array pra um valor entre -1 e 1
  Dessa forma quando estiver centralizado na linha o resultado é 0
  Estando deslocado para a direita o resultado é < 0
  e para a esquerda > 0
  */

  // Linha branca
  return (m_QtrArray.readLine(m_SensorWeights, QTR_EMITTERS_ON, true) - m_CenterPosition) / m_CenterPosition;

  // Linha preta
  //return (m_QtrArray.readLine(m_SensorWeights, QTR_EMITTERS_ON) - m_CenterPosition) / m_CenterPosition;
}

void PrestoSensoring::setSampleTimes(unsigned long leftSampleTime, unsigned long rightSampleTime)
{
  /*
  O tempo entre loops do código pode ser muito pequeno e a mesma marca pode ser lida várias vezes.
  Pra isso definimos esses tempos como o mínimo entre uma leitura e outra.
  */
  m_LeftSampleTime = leftSampleTime;
  m_RightSampleTime = rightSampleTime;
}

void PrestoSensoring::calibrate(Button commandButton, unsigned char statusLedPin)
{
  while(!commandButton.isPressed());

#ifdef DEBUG
  CurrentLogger->writeLine("Iniciando calibração");
#endif

  digitalWrite(statusLedPin, HIGH);

  while(!commandButton.isPressed())
  {
    m_QtrArray.calibrate();
    m_QtrRight.calibrate();
    m_QtrLeft.calibrate();
  }

  /*
  Calcula a média entre o máximo e o mínimo que foi lido nesse sensores, com isso temos
  uma referência para comparar se foi lido ou não, já que esses são sensores analógicos.
  */
  m_LeftSensorThreshold = (m_QtrLeft.getCalibratedMinimum(false)[0] + m_QtrLeft.getCalibratedMaximum(false)[0]) / 2;
  m_RightSensorThreshold = (m_QtrRight.getCalibratedMinimum(false)[0] + m_QtrRight.getCalibratedMaximum(false)[0]) / 2;

#ifdef DEBUG
  CurrentLogger->writeLine("Calibração concluída");
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

  // Usa os tempos de amostragem para garantir que não estamos lendo a mesma marca várias vezes
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

  // Interseção da linha na pista
  if(leftMark && rightMark)
  {
    // Não faz nada
  }
  // Marca de inicio/fim de prova
  else if(!leftMark && rightMark)
  {
    m_RightCount++;
  }
  // Marca de inicio/fim de curva
  else if(leftMark && !rightMark)
  {
    m_InCurve = !m_InCurve;
  }
}

bool PrestoSensoring::shouldStop(unsigned int rightMarks)
{
  if(m_RightCount >= rightMarks)
  {
    return true;
  }
  return false;
}
