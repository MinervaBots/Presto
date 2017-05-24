#ifndef BufferLogger_hpp
#define BufferLogger_hpp

#include "Logger.hpp"

class BufferLogger : public Logger
{
public:
  BufferLogger(unsigned int bufferSize);
  ~BufferLogger();
  void write(const char* format, ...);
  void writeLine(const char* format, ...);
  virtual void flush();
  const char *getBuffer() { return m_LogsBuffer; }
private:
  char *m_LogsBuffer;
  unsigned int m_BufferPosition;
  unsigned int m_BufferSize;
};

#endif // BufferLogger_hpp
