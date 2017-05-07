#ifndef SimpleMovingAverageFilter_hpp
#define SimpleMovingAverageFilter_hpp

class SimpleMovingAverageFilter
{
public:
  SimpleMovingAverageFilter(unsigned int samplesCapacity);
  ~SimpleMovingAverageFilter();
  float run(float sample);

private:
  unsigned int m_SamplesCount;
  unsigned int m_SamplesCapacity;
  float *m_pSamples;
};

#endif //SimpleMovingAverageFilter_hpp
