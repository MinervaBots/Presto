#include "BufferLogger.hpp"
//#include <stdlib.h>

BufferLogger::BufferLogger(unsigned int bufferSize)
{
  m_BufferPosition = 0;
  m_BufferSize = bufferSize;
  m_LogsBuffer = new char[bufferSize];//(char*)malloc(bufferSize);
}

BufferLogger::~BufferLogger()
{
  delete[] m_LogsBuffer;
  //free(m_LogsBuffer);
}

void BufferLogger::Flush(void (*f)(char*))
{
  if(m_BufferPosition == 0)
    return;

  f(m_LogsBuffer);
  m_BufferPosition = 0;
  //memset(m_LogsBuffer, 0, m_BufferSize);
}

void BufferLogger::Write(char *text)
{
  unsigned int lenght;
  for (lenght = 0; lenght < m_BufferSize; lenght++)
  {
    if(text[lenght] == '\0')
      break;
  }
  memcpy(m_LogsBuffer + m_BufferPosition, text, lenght);
  m_BufferPosition += lenght;
  m_LogsBuffer[m_BufferPosition + 1] = '\0';
}

void BufferLogger::WriteLine(char *text)
{
  unsigned int lenght;
  for (lenght = 0; lenght < m_BufferSize; lenght++)
  {
    if(text[lenght] == '\0')
      break;
  }
  text[lenght] = '\n';
  text[lenght + 1] = '\0';
  Write(text);
}
