#ifndef SimpleMovingAverageFilter_hpp
#define SimpleMovingAverageFilter_hpp

#include "Filter.hpp"

class SimpleMovingAverageFilter : public Filter
{
public:
  enum FilterMode
  {
    // Modo Continuo (ou de Histórico) faz a média das ultimas 'm_SamplesCapacity' leituras para amenizar ruido.
    Continuous,

    Static,
  };
  SimpleMovingAverageFilter(unsigned int samplesCapacity, FilterMode mode = FilterMode::Static);
  ~SimpleMovingAverageFilter();
  void setInputSource(InputSource* pInputSource) { m_pInputSource = pInputSource; }
  void setMode(FilterMode mode);
  float getInput();

private:
  FilterMode m_Mode;
  InputSource* m_pInputSource;
  unsigned int m_SamplesCount;
  unsigned int m_SamplesCapacity;
  unsigned int m_NextReplaced;
  float *m_pSamples;

  float runContinuous();
  float runStatic();
};

#endif //SimpleMovingAverageFilter_hpp
