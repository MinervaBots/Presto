#include "WheelEncoder.hpp"
#include <Arduino.h>

namespace WheelEncoderPrivate
{
  // attachInterrupt só aceita métodos estáticos, então preciso manter uma referência
  // em algum lugar para acessar esses métodos de interrupção.
  //
  // Esse é o design pattern chamado de Singleton (https://pt.wikipedia.org/wiki/Singleton).
  // Deixo isso dentro desse namespace pra não ser acessível de fora desse escopo
  static WheelEncoder *m_pWheelEncoderInstance;
}

WheelEncoder::WheelEncoder(char leftTickPin, char rightTickPin, float wheelRadius, unsigned int ticksPerRevolution)
{
  // Salva a instância em escopo estático
  WheelEncoderPrivate::m_pWheelEncoderInstance = this;

  setWheelRadius(wheelRadius);
  setTicksPerRevolution(ticksPerRevolution);
  pinMode(leftTickPin, INPUT);
  pinMode(rightTickPin, INPUT);

  // Define as interrupções em CHANGE pois o tick é contado como uma mudança
  // ALTO -> Baixo ou BAIXO -> ALTO
  attachInterrupt(digitalPinToInterrupt(leftTickPin), leftTick, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightTickPin), rightTick, CHANGE);
}

void WheelEncoder::leftTick()
{
  WheelEncoderPrivate::m_pWheelEncoderInstance->m_DeltaLeftTickCount++;
}

void WheelEncoder::rightTick()
{
  WheelEncoderPrivate::m_pWheelEncoderInstance->m_DeltaRightTickCount++;
}

void WheelEncoder::update()
{
  // Distância = Rotações * Circunferência
  // Circunferência = 2 * PI * Raio
  // Rotações = Ticks / TotalTicks

  // Calcula a distância desde o ultimo update (delta)
  m_DeltaDistanceLeft = 2 * PI * m_WheelRadius * (m_DeltaLeftTickCount / m_TicksPerRevolution);
  m_DeltaDistanceRight = 2 * PI * m_WheelRadius * (m_DeltaRightTickCount / m_TicksPerRevolution);

  // Incrementa a distância para obter o total
  m_TotalDistanceLeft += m_DeltaDistanceLeft;
  m_TotalDistanceRight += m_DeltaDistanceRight;

  // Reseta os ticks para se ter um delta na proxima chamada
  m_DeltaLeftTickCount = 0;
  m_DeltaRightTickCount = 0;
}
