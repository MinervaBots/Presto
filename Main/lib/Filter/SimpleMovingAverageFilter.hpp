#ifndef SimpleMovingAverageFilter_hpp
#define SimpleMovingAverageFilter_hpp

#include "Filter.hpp"

class SimpleMovingAverageFilter : public Filter
{
public:
  enum FilterMode
  {
    // Modo Continuo (ou de Histórico) faz a média das ultimas 'm_SamplesCapacity' leituras para amenizar ruido.
    // Extremamente rápido para uma quantidade modesta de leituras (pretendo remover a array e o loop pra otimizar).
    // Mas pode causar lag se a variação do sistema for muito bruca. Um novo filtro de média ponderada pode amenizar isso.
    // Pode também propagar rúido das ultimas avaliações.
    Continuous,

    // Modo Estático, realiza a leitura 'm_SamplesCapacity' vezes e então faz a média disso.
    // É mais lento mas evita propagar erros e teoricamente não apresenta lag de histórico.
    // Se o sistema depender muito de velocidade esse modo não é recomendado.
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
