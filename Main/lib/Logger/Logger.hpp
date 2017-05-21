#ifndef Logger_hpp
#define Logger_hpp

#include <string.h>

class Logger;
static Logger *CurrentLogger;

class Logger
{
public:
  Logger() { CurrentLogger = this; }
  virtual void Write(const char* format, ...) = 0;
  virtual void WriteLine(const char* format, ...) = 0;
};

#endif // Logger_hpp
