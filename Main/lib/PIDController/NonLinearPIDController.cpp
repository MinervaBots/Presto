#include "NonLinearPIDController.hpp"
#include <Arduino.h>

void NonLinearPIDController::setNonLinearConstanteCoeficients(float nonLinearProportionalCoeficient0,
  float nonLinearProportionalCoeficient1, float nonLinearIntegrativeCoeficient0,
  float nonLinearIntegrativeCoeficient1, float nonLinearDerivativeCoeficient)
{
  m_NonLinearProportionalCoeficient0 = nonLinearProportionalCoeficient0;
  m_NonLinearProportionalCoeficient1 = nonLinearProportionalCoeficient1;
  m_NonLinearIntegrativeCoeficient0 = nonLinearIntegrativeCoeficient0;
  m_NonLinearIntegrativeCoeficient1 = nonLinearIntegrativeCoeficient1;
  m_NonLinearDerivativeCoeficient = nonLinearDerivativeCoeficient;

  calculateTunings();
}

void NonLinearPIDController::setMaxError(float maxError)
{
  m_MaxError = maxError;
  calculateTunings();
}

void NonLinearPIDController::setSampleTime(int newSampleTime)
{
  PIDController::setSampleTime(newSampleTime);
  calculateTunings();
}

void NonLinearPIDController::setTunings(float proportionalConstant, float integralConstant, float derivativeConstant)
{
  PIDController::setTunings(proportionalConstant, integralConstant, derivativeConstant);
  calculateTunings();
}

void NonLinearPIDController::calculateTunings()
{
    float Kp0 = m_NonLinearProportionalCoeficient0 * m_ProportionalConstant;
    float Kp1 = m_NonLinearProportionalCoeficient1 * m_ProportionalConstant;

    float Ki0 = m_NonLinearIntegrativeCoeficient0 * m_IntegralConstant;
    float Ki1 = m_NonLinearIntegrativeCoeficient1 * m_IntegralConstant;

    float Kd = m_NonLinearDerivativeCoeficient * m_DerivativeConstant;

    float sqMaxError = pow(m_MaxError, 2);

    m_Alpha0 = (Kp1 - Kp0) / sqMaxError;
    m_Beta = Kp0;

    m_Alpha1 = (Kd) / sqMaxError;

    m_Gama = Ki0;
    m_Sigma = log(Ki1) / (log(Ki0) * pow(Ki0 * m_MaxError, 2));
}

float NonLinearPIDController::run(float input)
{
  unsigned long deltaTime;
	if (checkTime(&deltaTime))
	{
		return m_LastOutput;
	}

	float error = m_SetPoint - input;
  float squaredError = pow(error, 2);

  float nonLinearProportional = m_Alpha0 * squaredError + m_Beta;
  float nonLinearDerivative = m_Alpha1 * squaredError;
  float nonLinearIntegrative = m_Gama * exp(-m_Sigma * pow(m_Gama * error, 2));

  return compute(input, deltaTime, nonLinearProportional, nonLinearIntegrative, nonLinearDerivative);
}
