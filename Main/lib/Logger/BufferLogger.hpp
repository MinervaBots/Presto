#ifndef BufferLogger_hpp
#define BufferLogger_hpp

#include "Logger.hpp"

class BufferLogger : public Logger
{
public:
  BufferLogger(unsigned int bufferSize);
  ~BufferLogger();
  void Write(char *text);
  void WriteLine(char *text);
  virtual void Flush(void (*f)(char*));

private:
  char *m_LogsBuffer;
  unsigned int m_BufferPosition;
  unsigned int m_BufferSize;
};

#endif // BufferLogger_hpp
