#include "SimpleMovingAverageFilter.hpp"
#include <stdlib.h>

SimpleMovingAverageFilter::SimpleMovingAverageFilter(unsigned int samplesCapacity, FilterMode mode) :
  m_SamplesCount(0),
  m_SamplesCapacity(samplesCapacity),
  m_NextReplaced(0)
{
  setMode(mode);
}

SimpleMovingAverageFilter::~SimpleMovingAverageFilter()
{
  free(m_pSamples);
}

void SimpleMovingAverageFilter::setMode(FilterMode mode)
{
  m_Mode = mode;
  if(m_Mode == FilterMode::Continuous && m_pSamples == nullptr)
  {
    m_pSamples = (float*)malloc (m_SamplesCapacity * sizeof(float));
  }
  else
  {
    free(m_pSamples);
  }
}

float SimpleMovingAverageFilter::getInput()
{
  if(m_Mode == FilterMode::Continuous)
  {
    return runContinuous();
  }
  return runStatic();
}

float SimpleMovingAverageFilter::runContinuous()
{
  if(++m_SamplesCount > m_SamplesCapacity)
  {
    m_SamplesCount = m_SamplesCapacity;
  }
  if(m_NextReplaced >= m_SamplesCapacity)
  {
    m_NextReplaced = m_SamplesCapacity;
  }

  float output = m_pInputSource->getInput();
  m_pSamples[m_NextReplaced++] = output;

  for(unsigned int i = 0; i < m_SamplesCount - 1; i++)
  {
    output += m_pSamples[i];
  }
  return output / m_SamplesCount;
}

float SimpleMovingAverageFilter::runStatic()
{
  float output = 0;
  for (m_SamplesCount = 0; m_SamplesCount < m_SamplesCapacity; m_SamplesCount++)
  {
    output += m_pInputSource->getInput();
  }
  return output / m_SamplesCapacity;
}
