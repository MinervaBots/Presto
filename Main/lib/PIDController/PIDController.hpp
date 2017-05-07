#ifndef PIDController_hpp
#define PIDController_hpp

#include "../SystemController/SystemController.hpp"

class PIDController : public SystemController
{
public:
	PIDController() {}
	PIDController(int sampleTime, float setPoint, float minOutput, float maxOutput,
										float proportionalConstant, float integralConstant, float derivativeConstant,
									SystemControllerDirection controllerDirection);

	void setSampleTime(int newSampleTime);
  void setOutputLimits(float min, float max);
  void setTunings(float proportionalConstant, float integralConstant, float derivativeConstant);

  virtual float run(float intput);

protected:
	float m_ProportionalConstant;
	float m_DerivativeConstant;
	float m_IntegralConstant;
	float m_IntegrativeTermSum;
	unsigned long m_Now;

	bool checkTime(unsigned long *pDeltaTime);
	float compute(float input, float proportionalConstant, float integralConstant, float derivativeConstant);
};

#endif // PIDController_hpp
