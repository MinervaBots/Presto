#ifndef Filter_hpp
#define Filter_hpp

#include "../InputSource/InputSource.hpp"

class Filter : public InputSource
{
public:
  virtual float getInput() = 0;
};

#endif //Filter_hpp
