#ifndef Logger_hpp
#define Logger_hpp

#include <string.h>

class Logger
{
public:
  Logger() { CurrentLogger = this; }
  virtual void Write(char *text) = 0;
  virtual void WriteLine(char *text) = 0;

  static Logger *CurrentLogger;
};

#endif // Logger_hpp
