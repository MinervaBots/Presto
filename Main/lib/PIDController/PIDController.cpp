#include "PIDController.hpp"
#include <MathHelper.h>
#include <Arduino.h>

PIDController::PIDController(int sampleTime, float setPoint, float minOutput, float maxOutput,
									float proportionalConstant, float integralConstant, float derivativeConstant,
								SystemControllerDirection controllerDirection)
{
	setSampleTime(sampleTime);
	setSetPoint(setPoint);
	setOutputLimits(minOutput, maxOutput);
	setTunings(proportionalConstant, integralConstant, derivativeConstant);
	setControllerDirection(controllerDirection);
}

void PIDController::setSampleTime(int newSampleTime)
{
  if (newSampleTime > 0 && m_SampleTime > 0)
	{
		float ratio = newSampleTime / m_SampleTime;
		m_IntegralConstant *= ratio;
		m_DerivativeConstant /= ratio;
  }
  SystemController::setSampleTime(newSampleTime);
}

void PIDController::setOutputLimits(float min, float max)
{
  SystemController::setOutputLimits(min, max);
  m_IntegrativeTermSum = clamp(m_IntegrativeTermSum, m_MinOutput, m_MaxOutput);
}

void PIDController::setTunings(float proportionalConstant, float integralConstant, float derivativeConstant)
{
	if (proportionalConstant < 0 || integralConstant < 0 || derivativeConstant < 0)
	{
    #ifdef USE_SERIAL
    Serial.println("[PIDController::setTunings]: As constantes devem ser valores apenas positivos\n" +
      "Use o modo de operação inverso");
    #endif
	}

	m_ProportionalConstant = proportionalConstant;
	// Essa converção não é necessária, mas permite que a gente entre com
	// valores de KI e KD em termos de 1/segundo
	float sampleTimeInSec = ((float) m_SampleTime) / 1000;
	m_IntegralConstant = integralConstant * sampleTimeInSec;
	m_DerivativeConstant = derivativeConstant / sampleTimeInSec;
	// A aplicação direta dos valores aqui nas constantes só é possivel
	// porque o tempo de avalição do PID é fixado. Isso evita também que a
	// multiplicação e principalmente a divisão tenham que ser feitas
	// cada vez que o PID é calculado.
	// tl;dr: deixa o código mais rápido e mais eficiente.

	if (m_ControllerDirection == SystemControllerDirection::Inverse)
	{
		m_ProportionalConstant *= -1;
		m_IntegralConstant *= -1;
		m_DerivativeConstant *= -1;
	}
}

float PIDController::run(float input)
{
	unsigned long deltaTime;
	if (checkTime(&deltaTime))
	{
		return m_LastOutput;
	}
	return compute(input, m_ProportionalConstant, m_IntegralConstant, m_DerivativeConstant);
}

bool PIDController::checkTime(unsigned long *pDeltaTime)
{
	m_Now = millis();
	*pDeltaTime = (m_Now - m_LastRunTime);
	return *pDeltaTime > m_SampleTime;
}

float PIDController::compute(float input, float proportionalConstant, float integralConstant, float derivativeConstant)
{
	float error = m_SetPoint - input;

	// Faz a derivada das entradas para evitar o "derivative kick", que
	// ocorre mudando o setPoint.
	// Não acontece em nenhum dos nossos projetos, mas é uma implementação
	// melhor e o custo computacional é identico
	float dInput = (input - m_LastInput);


	// Salva o valor acumulado do fator integrativo
	// Isso torna possivel mudar a constante integrativa sem gerar uma
	// mudança abruta na saída já que o acumulo dos erros não é mais multiplicado
	// pelo mesmo valor que antes.
	m_IntegrativeTermSum += error * integralConstant;
	// Restrige o erro acumulado pra evitar que este aumente indefinidamente
	// e extrapole os limites que de trabalho do nosso sistema.
	// Apesar da saída do sistema também ser limitado, a restrição do
	// acumulo é necessária para que o sistema responda imediatamente a uma
	// mudança na entrada e não tente compensar o integrativo desnecessariamente.
	m_IntegrativeTermSum = clamp(m_IntegrativeTermSum, m_MinOutput, m_MaxOutput);


	float output = proportionalConstant * error;	// Proporcional
	output += m_IntegrativeTermSum; 								// Integrativo
	output -= derivativeConstant * dInput; 				// Derivativo

	// Faz clamp da saída do PID também, pois os fatores proporcional e
	// derivativo também
	// podem fazer com que a saída extrapole o intervalo de trabalho do
	// sistema
	m_LastOutput = clamp(output, m_MinOutput, m_MaxOutput);
	m_LastRunTime = m_Now;
	m_LastInput = input;
	return m_LastOutput;
}
