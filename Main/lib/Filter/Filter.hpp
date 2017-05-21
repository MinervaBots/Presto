#ifndef Filter_hpp
#define Filter_hpp

#include "../InputSource/InputSource.hpp"

class Filter : public InputSource
{
public:
  virtual float getInput() = 0;
  virtual void setInputSource(InputSource* pInputSource) { m_pInputSource = pInputSource; }

protected:
  InputSource *m_pInputSource;
};

#endif //Filter_hpp
