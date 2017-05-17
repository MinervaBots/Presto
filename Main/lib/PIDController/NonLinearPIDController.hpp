#ifndef NonLinearPIDController_hpp
#define NonLinearPIDController_hpp

#include "PIDController.hpp"

class NonLinearPIDController : public PIDController
{
public:
  float run(float intput);
  void setMaxError(float maxError);
  void setSampleTime(int newSampleTime);
  void setTunings(float proportionalConstant, float integralConstant, float derivativeConstant);
  void setNonLinearConstanteCoeficients(float nonLinearProportionalCoeficient0, float nonLinearProportionalCoeficient1,
  float nonLinearIntegrativeCoeficient0, float nonLinearIntegrativeCoeficient1, float nonLinearDerivativeCoeficient);

private:
  float m_MaxError;
  float m_NonLinearProportionalCoeficient0;
  float m_NonLinearProportionalCoeficient1;
  float m_NonLinearIntegrativeCoeficient0;
  float m_NonLinearIntegrativeCoeficient1;
  float m_NonLinearDerivativeCoeficient;

  float m_Alpha0;
  float m_Beta;

  float m_Alpha1;

  float m_Gama;
  float m_Sigma;

  void calculateTunings();
};

#endif // NonLinearPIDController_hpp
