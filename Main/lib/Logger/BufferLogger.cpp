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

void BufferLogger::Flush()
{
  m_BufferPosition = 0;
  //memset(m_LogsBuffer, 0, m_BufferSize);
}

void BufferLogger::Write(const char* format, ...)
{
  /*
  char msg[100];
  va_list args;
  va_start (args, format);
  vsnprintf (msg, 256, format, args);
  //perror (msg);
  va_end (args);

  unsigned int lenght;
  for (lenght = 0; lenght < m_BufferSize; lenght++)
  {
    if(msg[lenght] == '\0')
      break;
  }
  memcpy(m_LogsBuffer + m_BufferPosition, msg, lenght);
  */

  // Dessa forma eu não tenho certeza se funciona. O jeito de cima é "garantido" (não foi testado).
  // Mas aqui a gente não precisa se preocupar muito em alocar um novo array sempre que mandar uma mensagem,
  // e podemos ter mensagens grandes por vez (256 no momento)
  va_list args;
  va_start (args, format);
  vsnprintf (&m_LogsBuffer[m_BufferPosition], 256, format, args);
  //perror (msg);
  va_end (args);

  unsigned int lenght;
  for (lenght = 0; lenght < m_BufferSize; lenght++)
  {
    if(m_LogsBuffer[lenght] == '\0')
      break;
  }

  m_BufferPosition += lenght + 1;
  m_LogsBuffer[m_BufferPosition] = '\0';
}

void BufferLogger::WriteLine(const char* format, ...)
{
  // TODO - Verificar se a formatação vai funcionar
  // Se não preciso fazer aqui o mesmo que fiz em 'Write'
  Write(format);
  m_LogsBuffer[m_BufferPosition] = '\n';
  m_LogsBuffer[m_BufferPosition + 1] = '\0';
}
