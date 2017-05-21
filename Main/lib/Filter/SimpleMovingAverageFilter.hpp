#ifndef SimpleMovingAverageFilter_hpp
#define SimpleMovingAverageFilter_hpp

#include "Filter.hpp"

// Filtro de média móvel (Progressiva ou de Histórico) faz a média das ultimas
// 'm_SamplesCapacity' leituras para amenizar ruido.
// Extremamente rápido mas ocupa uma quantidade maior de memória.
// Mas pode causar lag se a variação do sistema for muito bruca, ou se o tamanho do histórico for muito grande
// Pode também propagar rúido das ultimas avaliações.
//
// Um novo filtro de média ponderada pode amenizar isso.
class SimpleMovingAverageFilter : public Filter
{
public:
  SimpleMovingAverageFilter(unsigned int samplesCapacity);
  ~SimpleMovingAverageFilter();
  float getInput();
  void clear();

private:
  InputSource* m_pInputSource;

  unsigned int m_SamplesCount;
  unsigned int m_SamplesCapacity;
  unsigned int m_Position;
  float *m_pSamples;
  float m_Sum;
};

#endif //SimpleMovingAverageFilter_hpp
