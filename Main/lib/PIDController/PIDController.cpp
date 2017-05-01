#include "PIDController.hpp"
#include <MathHelper.h>
#include <Arduino.h>


PIDController::PIDController()
{

}

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
	// valores
	// de KI e KD em termos de 1/segundo
	float sampleTimeInSec = ((float) m_SampleTime) / 1000;
	m_IntegralConstant = integralConstant * sampleTimeInSec;
	m_DerivativeConstant = derivativeConstant / sampleTimeInSec;
	// A aplicação direta dos valores aqui nas constantes só é possivel
	// porque o
	// tempo de avalição do PID é fixado. Isso evita também que a
	// multiplicação
	// E principalmente a divisão tenham que ser feitas cada vez que o PID é
	// calculado
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
	long now = millis();
	unsigned long deltaTime = (now - m_LastRunTime);
	if (deltaTime < m_SampleTime)
	{
		return m_LastOutput;
	}

	float error = m_SetPoint - input;

	// Salva o valor acumulado do fator integrativo
	// Isso torna possivel mudar a constante integrativa sem gerar uma
	// mudança abruta na saída
	// já que o acumulo dos erros não é mais multiplicado pelo mesmo valor
	// que antes
	m_IntegrativeTermSum += error * m_IntegralConstant;
	// Faz o clamp disso pra evitar que o erro se acumule indefinidamente]
	// e extrapole os limites que o nosso sistema usa.
	// Apesar da saída do sistema também ser limitado, precisa fazer do
	// acumulo dos erros
	// pra que o sistema responda imediatamente a uma mudança na entrada e
	// não tente compensar o integrativo desnecessariamente
	m_IntegrativeTermSum = clamp(m_IntegrativeTermSum, m_MinOutput, m_MaxOutput);

	// Faz a derivada das entradas para evitar o "derivative kick", que
	// ocorre mudando o setPoint
	// Não acontece em nenhum dos nossos projetos, mas é uma implementação
	// melhor,
	// e o custo computacional é identico
	float dInput = (input - m_LastInput);

	float output = m_ProportionalConstant * error; // Proporcional
	output += m_IntegrativeTermSum; // Integrativo
	output -= m_DerivativeConstant * dInput; // Derivativo

	// Faz clamp da saída do PID também, pois os fatores proporcional e
	// derivativo também
	// podem fazer com que a saída extrapole o intervalo de trabalho do
	// sistema
	output = clamp(output, m_MinOutput, m_MaxOutput);

	m_LastInput = input;
	m_LastRunTime = now;
	m_LastOutput = output;

	return output;
}
