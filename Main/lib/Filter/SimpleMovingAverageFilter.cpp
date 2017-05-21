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
  // Só vamos alocar se estivermos mudando para o modo contínuo e já não temos esse espaço alocado
  // Tenho que verificar o que é pior, alocar e desalocar sempre que mudar de modo (gasta CPU),
  // ou só alocar uma vez e deixar de lado (ocupa memória desnecessáriamente).
  //
  // Na prática, nenhum dos nossos casos de uso iria trocar de modo em tempo de execução então
  // não faz diferença ¯\_(ツ)_/¯
  if(mode == FilterMode::Continuous && m_pSamples == nullptr)
  {
    // Trocando pra esse modo zera o contador pra não causar erros no calculo
    m_SamplesCount = 0;
    m_pSamples = (float*)malloc (m_SamplesCapacity * sizeof(float));
  }
  // Também só vamos liberar a memória se estivermos indo pro modo estático e se tivermos algo pra liberar
  else if(mode == FilterMode::Static && m_pSamples != nullptr)
  {
    free(m_pSamples);
  }
  m_Mode = mode;
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
  // Incrementa a quantidade de leituras (amostras) que temos agora
  // e limita pra que não saia dos limites do array
  if(++m_SamplesCount > m_SamplesCapacity)
  {
    m_SamplesCount = m_SamplesCapacity;
  }

  // Substitui algum valor (o mais antigo) no array de leituras, pela leitura atual
  m_pSamples[m_NextReplaced] = m_pInputSource->getInput();

  // Incrementa a próxima posição a ser substituida (agora está é a leitura mais antiga).
  // Se isso for além dos limites do array, volta pra primeira (ping-pong clamp).
  if(++m_NextReplaced > m_SamplesCapacity)
  {
    m_NextReplaced = 0;
  }

  // Itera pelo array e soma todos os valores na variável 'output'
  float output = 0;
  for(unsigned int i = 0; i < m_SamplesCount; i++)
  {
    output += m_pSamples[i];
  }
  // Calcula a média dividindo pelo número de leituras
  return output / m_SamplesCount;
}

float SimpleMovingAverageFilter::runStatic()
{
  // Aqui é bem simples e direto
  // Só repete a leitura 'm_SamplesCapacity' vezes e soma a variável 'output'
  float output = 0;
  for (m_SamplesCount = 0; m_SamplesCount < m_SamplesCapacity; m_SamplesCount++)
  {
    output += m_pInputSource->getInput();
  }
  // Faz a média dividindo pelo número de leituras que ele realizou
  return output / m_SamplesCapacity;
}
