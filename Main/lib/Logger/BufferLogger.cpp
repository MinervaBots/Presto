#include "BufferLogger.hpp"
//#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

BufferLogger::BufferLogger(unsigned int bufferSize)
{
  m_BufferPosition = 0;
  m_BufferSize = bufferSize;
  m_LogsBuffer = new char[bufferSize];//(char*)malloc(bufferSize);
}

BufferLogger::~BufferLogger()
{
  delete [] m_LogsBuffer;
  //free(m_LogsBuffer);
}

void BufferLogger::flush()
{
  m_BufferPosition = 0;
  memset(m_LogsBuffer, '\0', m_BufferSize);
}

void BufferLogger::write(const char* format, ...)
{
  /*
  O tamanho disponível é o total do buffer menos o que já tá ocupado.
  Subtrai 1 aqui pra garantir que vai ter espaço pro marcador de fim de string.
  */
  unsigned int availabeSize = m_BufferSize - m_BufferPosition - 1;

  va_list args;
  va_start (args, format);
  vsnprintf (&m_LogsBuffer[m_BufferPosition], availabeSize, format, args);
  va_end (args);

  unsigned int lenght;
  for (lenght = 0; lenght < availabeSize; lenght++)
  {
    if(m_LogsBuffer[m_BufferPosition + lenght] == '\0')
      break;
  }
  // Incrementa o tamanho da string que foi adicionado
  m_BufferPosition += lenght;

  /*
  Posiciona o marcador de fim de string logo no final do que foi adicionado agora
  Se algo for escrito depois vai sobrescrever esse marcador.

  Por causa de flush(), teoricamente tudo depois disso é '\0' então não precisa
  se preocupar com marcador de fim de string.
  */
  //m_LogsBuffer[m_BufferPosition] = '\0';
}

void BufferLogger::writeLine(const char* format, ...)
{
  // TODO - Verificar se a formatação vai funcionar
  // Se não preciso fazer aqui o mesmo que fiz em 'Write'
  write(format);
  // Sobrescreve o marcador de final de string por um de final de linha
  m_LogsBuffer[m_BufferPosition] = '\n';

  /*
  Por causa de flush(), teoricamente tudo depois disso é '\0' então não precisa
  se preocupar com marcador de fim de string.
  */
}
