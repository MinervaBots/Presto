#ifndef LowPassFilter_hpp
#define LowPassFilter_hpp

class LowPassFilter
{
public:
  LowPassFilter(unsigned int samplesCapacity);
  ~LowPassFilter();
  float run(float sample);

private:
  unsigned int m_SamplesCount;
  unsigned int m_SamplesCapacity;
  float *m_pSamples;
};

#endif //LowPassFilter_hpp
