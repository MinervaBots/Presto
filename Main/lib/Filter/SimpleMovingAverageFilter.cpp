#include "SimpleMovingAverageFilter.hpp"
#include <stdlib.h>
//#include <stdio.h>

SimpleMovingAverageFilter::SimpleMovingAverageFilter(unsigned int samplesCapacity) :
  m_SamplesCount(0),
  m_SamplesCapacity(samplesCapacity),
  m_Position(0),
  m_Sum(0)
{
  m_pSamples = new float[m_SamplesCapacity];
  //m_pSamples = (float*)calloc (m_SamplesCapacity, sizeof(float));
}

SimpleMovingAverageFilter::~SimpleMovingAverageFilter()
{
  delete [] m_pSamples;
  //free(m_pSamples);
}

void SimpleMovingAverageFilter::clear()
{
  m_Sum = 0;
  m_SamplesCount = 0;
  m_Position = 0;
  // Da forma que foi implementado isso não é necessário,
  // mas é uma boa prática limpar tudo. Manter ou não?
  //memset(m_pSamples[0], 0,  sizeof(float) * m_SamplesCapacity);
}

float SimpleMovingAverageFilter::getInput()
{
  // Incrementa a quantidade de leituras (amostras) que temos agora
  // e limita pra que não saia dos limites do array
  if(++m_SamplesCount > m_SamplesCapacity)
  {
    m_SamplesCount = m_SamplesCapacity;
    m_Sum -= m_pSamples[m_Position];
  }

  // Faz a leitura
  float sample = m_pInputSource->getInput();

  // Substitui algum valor (o mais antigo) no array de leituras, pela leitura atual
  m_pSamples[m_Position] = sample;

  // Adiciona ao somatório
  m_Sum += sample;
  // Incrementa a próxima posição a ser substituida (agora está é a leitura mais antiga).
  // Se isso for além dos limites do array, volta pra primeira (ping-pong clamp).
  if(++m_Position > m_SamplesCapacity)
  {
    m_Position = 0;
  }
  // Calcula a média dividindo pelo número de leituras
  return m_Sum / m_SamplesCount;
}
