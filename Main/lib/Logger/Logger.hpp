#ifndef Logger_hpp
#define Logger_hpp

#include <string.h>
//#include "../CompilerDefinitions.h"

class Logger;
static Logger *CurrentLogger;

class Logger
{
public:
  Logger() { CurrentLogger = this; }
  virtual void write(const char* format, ...) = 0;
  virtual void writeLine(const char* format, ...) = 0;
};

#endif // Logger_hpp
