#ifndef SimpleMovingAverageFilter_hpp
#define SimpleMovingAverageFilter_hpp

#include "Filter.hpp"

class SimpleMovingAverageFilter : public Filter
{
public:
  SimpleMovingAverageFilter(unsigned int samplesCapacity);
  ~SimpleMovingAverageFilter();
  float run(float sample);

private:
  unsigned int m_SamplesCount;
  unsigned int m_SamplesCapacity;
  unsigned int m_NextReplaced;
  float *m_pSamples;
};

#endif //SimpleMovingAverageFilter_hpp
