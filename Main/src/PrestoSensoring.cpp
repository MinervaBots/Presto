#include "PrestoSensoring.hpp"
#include "../lib/Logger/Logger.hpp"

PrestoSensoring::PrestoSensoring()
{

}

void PrestoSensoring::setLeftSensor(unsigned char sensorPin, unsigned long sampleTime, unsigned long timeout)
{
  m_LeftSensorPin = sensorPin;
  /*
  O tempo entre loops do código pode ser muito pequeno e a mesma marca pode ser lida várias vezes.
  Pra isso definimos esses tempos como o mínimo entre uma leitura e outra.
  */

  m_LeftSampleTime = sampleTime;
  m_LeftSensorTimeout = timeout;
}

void PrestoSensoring::setRightSensor(unsigned char sensorPin, unsigned long sampleTime, unsigned long timeout)
{
  m_RightSensorPin = sensorPin;
  /*
  O tempo entre loops do código pode ser muito pequeno e a mesma marca pode ser lida várias vezes.
  Pra isso definimos esses tempos como o mínimo entre uma leitura e outra.
  */

  m_RightSampleTime = sampleTime;
  m_RightSensorTimeout = timeout;
}

void PrestoSensoring::setSensorArray(unsigned char* arrayPins, unsigned char pinsCount, unsigned long timeout)
{
  m_SensorArrayCount = pinsCount;
  m_SensorArrayTimeout = timeout;
  //Serial.println(m_SensorArrayTimeout);
  m_SensorArrayPins = new unsigned char[pinsCount];
  unsigned char i;
  for (i = 0; i < pinsCount; i++)
  {
    m_SensorArrayPins[i] = arrayPins[i];
  }

  /*
  Calcula qual o valor central que o array retorna
  Como QTRSensor retorna o número [0..n-1] do sensor * 1000:
  */

  //Serial.println(pinsCount);
  m_CenterPosition = (float)(pinsCount - 1) / 2;
  //Serial.println(m_CenterPosition);
}

float PrestoSensoring::getInput()
{
  unsigned long avg = 0; // this is for the weighted total, which is long
                     // before division
  unsigned int sum = 0; // this is for the denominator which is <= 64000

  int onLine = -1;
  for (unsigned char i = 0; i < m_SensorArrayCount; i++)
  {
    unsigned int value = readPin(m_SensorArrayPins[i], m_SensorArrayTimeout);
    value = clamp(value, m_SensorArrayMinReading[i], m_SensorArrayMaxReading[i]);

    if(m_LineColor == LineColor::White)
    {
      value = 1000 - value;
    }
    /*
    Serial.print(m_SensorArrayPins[i]);
    Serial.print(": ");
    Serial.println(value);
    */
    /*
    [TODO]
    Bolar algum jeito melhor de eliminar ruido.
    Se for necessário
    */
    if(value > 20)
    {
      avg += value * i;
      sum += value;
    }
    // Por enquanto nenhum sensor está na linha
    if(onLine == -1 && value >= 500)
    {
      onLine = 1;
    }
  }

  if(onLine == -1)
  {
    /*
    Conserva a ultima direção
    */
    if(m_LastValue < 0)
        return -1;
    else if(m_LastValue > 0)
        return 1;
  }
  /*
  Serial.print("sum: ");
  Serial.println(sum);
  Serial.print("avg: ");
  Serial.println(avg);
  */
  /*
  Normaliza a saída do array pra um valor entre -1 e 1
  Dessa forma quando estiver centralizado na linha o resultado é 0
  Estando deslocado para a direita o resultado é < 0
  e para a esquerda > 0
  */
  if(sum == 0)
  {
    return 0;
  }
  m_LastValue = (avg / (float)sum) - m_CenterPosition;

  /*
  Se tem um desvio grande da linha estamos em uma curva
  */
  m_InCurve = (abs(m_LastValue) > 0.3);

  return m_LastValue;
}

unsigned int PrestoSensoring::readPin(unsigned char pin, unsigned long timeout)
{
  unsigned int value = timeout;
  pinMode(pin, OUTPUT);         // make sensor line an output
  digitalWrite(pin, HIGH);      // drive sensor line high
  delayMicroseconds(10);        // charge lines for 10 us
  digitalWrite(pin, HIGH);      // important: disable internal pull-up!
  pinMode(pin, INPUT);          // make sensor line an input

  unsigned long startTime = micros();
  while (micros() - startTime < timeout)
  {
    unsigned int time = micros() - startTime;
    if (digitalRead(pin) == LOW && time < value)
    {
      value = time;
    }
  }
  return value;
}

void PrestoSensoring::calibrate(Button commandButton, unsigned char statusLedPin)
{
  pinMode(statusLedPin, OUTPUT);

  while(!commandButton.isPressed());
#ifdef DEBUG2
  //CurrentLogger->writeLine("Iniciando calibração");
#endif
  digitalWrite(statusLedPin, HIGH);
  delay(250);

  if(m_SensorArrayMinReading == nullptr)
  {
    m_SensorArrayMinReading = new unsigned int[m_SensorArrayCount];
    memset(m_SensorArrayMinReading, (unsigned int)INFINITY, sizeof(int) * m_SensorArrayCount);
    m_SensorArrayMaxReading = new unsigned int[m_SensorArrayCount];
    memset(m_SensorArrayMaxReading, 0, sizeof(int) * m_SensorArrayCount);
  }

  while(!commandButton.isPressed())
  {
    unsigned int value;
    for (unsigned char i = 0; i < m_SensorArrayCount; i++)
    {
      value = readPin(m_SensorArrayPins[i], m_SensorArrayTimeout);
      if(value < m_SensorArrayMinReading[i])
      {
        m_SensorArrayMinReading[i] = value;
      }
      else if(value > m_SensorArrayMaxReading[i])
      {
        m_SensorArrayMaxReading[i] = value;
      }
    }

    value = readPin(m_LeftSensorPin, m_LeftSensorTimeout);
    if(value < m_LeftMinReading)
    {
      m_LeftMinReading = value;
    }
    else if(value > m_LeftMaxReading)
    {
      m_LeftMaxReading = value;
    }

    value = readPin(m_RightSensorPin, m_RightSensorTimeout);
    if(value < m_RightMinReading)
    {
      m_RightMinReading = value;
    }
    else if(value > m_RightMaxReading)
    {
      m_RightMaxReading = value;
    }
  }

  /*
  Calcula a média entre o máximo e o mínimo que foi lido nesse sensores, com isso temos
  uma referência para comparar se foi lido ou não, já que esses são sensores analógicos.
  */
  //for (unsigned char i = 0; i < m_SensorArrayCount; i++)
  {
    //Serial.println(m_SensorArrayMinReading[i]);
    //Serial.println(m_SensorArrayMaxReading[i]);
  }

#ifdef DEBUG2
  CurrentLogger->writeLine("Calibração concluída");
#endif
  digitalWrite(statusLedPin, LOW);
  delay(250);

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
    value = readPin(m_LeftSensorPin, m_LeftSensorTimeout);
    value = clamp(value, m_LeftMinReading, m_LeftMaxReading);
    if(m_LineColor == LineColor::White)
    {
      value = 1000 - value;
    }
    leftMark = value > 500;
  }

  if((now - m_LastRun) > m_RightSampleTime)
  {
    value = readPin(m_RightSensorPin, m_RightSensorTimeout);
    value = clamp(value, m_RightMinReading, m_RightMaxReading);
    if(m_LineColor == LineColor::White)
    {
      value = 1000 - value;
    }
    rightMark = value > 500;
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
    /*
    Essa linha está comentada pois estamos explorando a possibilidade de usar a
    leitura dos sensores como indicativo de curva. Assim podemos controlar a
    velocidade de forma mais dinâmica.
    */
    //m_InCurve = !m_InCurve;
  }
}

bool PrestoSensoring::shouldStop(unsigned int rightMarks)
{
  if(m_RightCount >= rightMarks)
  {
    //Serial.println("true_");
    return true;
  }
  return false;
}

unsigned int PrestoSensoring::clamp(unsigned int value, unsigned int min, unsigned int max)
{
  unsigned int denominator = (max - min);
  signed int x = 0;
  if(denominator != 0)
    x = (((signed long)constrain(value, min, max)) - min) * 1000 / denominator;

  if(x < 0)
      return 0;
  else if(x > 1000)
      return 1000;

  return x;
}
