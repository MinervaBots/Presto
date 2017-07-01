#ifndef Definitions_h
#define Definitions_h

#include <stdarg.h>

// Templates
template <typename T, unsigned S> inline unsigned arraySize(const T (&v)[S]) { return S; }


// Macros


// Defines
#define NOT_USED 255


/*
#define PRINTF_BUF 80 // define the tmp buffer size (change if desired)
void printf(HardwareSerial serial, const char *format, ...)
{
  char buf[PRINTF_BUF];
  va_list ap;
  va_start(ap, format);
  vsnprintf(buf, sizeof(buf), format, ap);
  for(char *p = &buf[0]; *p; p++) // emulate cooked mode for newlines
  {
    if(*p == '\n')
      serial.write('\r');
    serial.write(*p);
  }
  va_end(ap);
}
*/

#endif //Definitions_h
