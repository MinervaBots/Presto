#ifndef Filter_hpp
#define Filter_hpp

class Filter
{
public:
  virtual float run(float sample) = 0;
};

#endif //Filter_hpp
