#ifndef PIDController_hpp
#define PIDController_hpp

#include "../SystemController/SystemController.hpp"

class PIDController : public SystemController
{
public:
	PIDController() : PIDController(0, 0, 0, 255, 0, 0, 0, SystemControllerDirection::Direct) {}
	PIDController(int sampleTime, float setPoint, float minOutput, float maxOutput,
										float proportionalConstant, float integralConstant, float derivativeConstant,
									SystemControllerDirection controllerDirection);

	virtual void setSampleTime(int newSampleTime);
  void setOutputLimits(float min, float max);
  virtual void setTunings(float proportionalConstant, float integralConstant, float derivativeConstant);
  virtual float run(float intput);

	float getProportionalConstant() { return m_ProportionalConstant; }
	float getDerivativeConstant() { return m_DerivativeConstant; }
	float getIntegralConstant() { return m_IntegralConstant; }
protected:
	float m_ProportionalConstant;
	float m_DerivativeConstant;
	float m_IntegralConstant;
	float m_IntegrativeTermSum;

  float m_LastOutput;
  float m_LastInput;
	float m_LastError;

	float compute(float input, unsigned long deltaTime, float proportionalConstant, float integralConstant, float derivativeConstant);
};

#endif // PIDController_hpp
