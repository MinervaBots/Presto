#ifndef PIDController_hpp
#define PIDController_hpp

#include "../SystemController/SystemController.hpp"

class PIDController : public SystemController
{
public:
	PIDController();
	PIDController(int sampleTime, float setPoint, float minOutput, float maxOutput,
										float proportionalConstant, float integralConstant, float derivativeConstant,
									SystemControllerDirection controllerDirection);

	void setSampleTime(int newSampleTime);
  void setOutputLimits(float min, float max);
  void setTunings(float proportionalConstant, float integralConstant, float derivativeConstant);

  float run(float intput);

private:
	float m_IntegralConstant;
	float m_DerivativeConstant;
	float m_IntegrativeTermSum;
	float m_ProportionalConstant;
};

#endif // PIDController_hpp
