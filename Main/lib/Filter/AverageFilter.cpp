#include "AverageFilter.hpp"


float AverageFilter::getInput()
{
  // Só pra evitar um zero division error
  if(m_NumberOfSamples == 0)
    return 0;

  // Zera a soma que tinhamos antes
  float sum = 0;

  /*
  Aqui é bem simples e direto
  Só repete a leitura 'm_SamplesCapacity' vezes e soma a variável 'sum'
  */
  for (unsigned int i = 0; i < m_NumberOfSamples; i++)
  {
    sum += m_pInputSource->getInput();
  }
  
  // Faz a média dividindo pelo número de leituras que ele realizou
  return sum / m_NumberOfSamples;
}
