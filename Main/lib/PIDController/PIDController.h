#ifndef PIDController_h
#define PIDController_h

#include "../SystemController/SystemController.h"

class PIDController : public SystemController
{
public:
	PIDController();
	PIDController(int sampleTime, float setPoint, float minOutput, float maxOutput,
										float proportionalConstant, float integralConstant, float derivativeConstant,
									SystemControllerDirection controllerDirection);
	~PIDController();

	void setSampleTime(int newSampleTime);
  void setOutputLimits(float min, float max);
  void setTunings(float proportionalConstant, float integralConstant, float derivativeConstant);

  float run(float intput);

private:
	float _integralConstant;
	float _derivativeConstant;
	float _integrativeTermSum;
	float _proportionalConstant;
};

#endif // PIDController_h
