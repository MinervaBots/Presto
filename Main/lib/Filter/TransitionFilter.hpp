#ifndef TransitionFilter_hpp
#define TransitionFilter_hpp

#include "Filter.hpp"
#include "../MathHelper/MathHelper.h"

typedef float (*TransitionFunction)(float, float, float);

class TransitionFilter : public Filter
{
public:
  TransitionFilter(float blendingFactor, TransitionFunction function) :
  m_BlendingFactor(blendingFactor), m_LastValue(0),  m_Function(function) {}
  float getInput() { return (m_LastValue = m_Function(m_pInputSource->getInput(), m_LastValue, m_BlendingFactor)); }

  // 0 -> Valor atual
  // 1 -> m_LastValue
  // Entre 0 e 1 -> Interpolação
  void setBledingFactor(float blendingFactor) { m_BlendingFactor = blendingFactor; }
  void setTransitionFunction(TransitionFunction function) { m_Function = function; }

private:
  float m_BlendingFactor;
  float m_LastValue;
  TransitionFunction m_Function;
};

#endif //TransitionFilter_hpp
