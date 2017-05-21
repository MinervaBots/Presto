#ifndef BufferLogger_hpp
#define BufferLogger_hpp

#include "Logger.hpp"

class BufferLogger : public Logger
{
public:
  BufferLogger(unsigned int bufferSize);
  ~BufferLogger();
  void Write(const char* format, ...);
  void WriteLine(const char* format, ...);
  virtual void Flush();
  const char *getBuffer() { return m_LogsBuffer; }
private:
  char *m_LogsBuffer;
  unsigned int m_BufferPosition;
  unsigned int m_BufferSize;
};

#endif // BufferLogger_hpp
