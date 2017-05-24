#ifndef AverageFilter_hpp
#define AverageFilter_hpp

#include "Filter.hpp"

/*
Modo Estático, realiza a leitura 'm_NumberOfSamples' vezes e então faz a média disso.
É mais lento mas evita propagar erros e teoricamente não apresenta lag de histórico.
Se o sistema depender muito de velocidade esse modo não é recomendado.
Em contrapartida não usa nenhuma memória extra com arrays e outras variáveis.
*/
class AverageFilter : public Filter
{
public:
  AverageFilter(unsigned int numberOfSamples) { setNumberOfSamples(numberOfSamples); }
  float getInput();

  void setNumberOfSamples(unsigned int numberOfSamples) { m_NumberOfSamples = numberOfSamples; }

private:
  unsigned int m_NumberOfSamples;
};

#endif // AverageFilter_hpp
