#ifndef InputSource_h
#define InputSource_h

class InputSource
{
public:
  virtual ~InputSource();
  virtual float getInput() = 0;
};

#endif // InputSource_h
