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


  float Kp0 = m_NonLinearProportionalCoeficient0 * m_ProportionalConstant;
  float Kp1 = m_NonLinearProportionalCoeficient1 * m_ProportionalConstant;

  float Ki0 = m_NonLinearIntegrativeCoeficient0 * m_IntegralConstant;
  float Ki1 = m_NonLinearIntegrativeCoeficient1 * m_IntegralConstant;

  float Kd = m_NonLinearDerivativeCoeficient * m_DerivativeConstant;


  float alpha0 = (Kp1 - Kp0) / pow(m_MaxError, 2);
  float beta = Kp0;
  float nonLinearProportional = alpha0 * squaredError + beta;

  // Isso provavelmete está errado
  // squaredError deve ser m_MaxError
  // Dessa forma alpha1 = Kd
  // TODO - Pesquisar mais sobre isso
  float alpha1 = Kd / squaredError;
  float nonLinearDerivative = alpha1 * squaredError;

  float gama = Ki0;
  float sigma = log(Ki1) / (log(Ki0) * pow(Ki0 * m_MaxError, 2));
  float nonLinearIntegrative = gama * exp(-sigma * pow(gama * error, 2));

  return compute(input, nonLinearProportional, nonLinearIntegrative, nonLinearDerivative);
}
