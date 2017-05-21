#ifndef Logger_hpp
#define Logger_hpp

#include <string.h>

class Logger;

namespace Log
{
    static Logger *CurrentLogger;
}

class Logger
{
public:
  Logger() { Log::CurrentLogger = this; }
  virtual void Write(const char* format, ...) = 0;
  virtual void WriteLine(const char* format, ...) = 0;
};

#endif // Logger_hpp
