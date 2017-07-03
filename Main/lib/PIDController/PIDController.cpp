#include "PIDController.hpp"
#include "../MathHelper/MathHelper.h"

PIDController::PIDController(int sampleTime, float setPoint, float minOutput, float maxOutput,
									float proportionalConstant, float integralConstant, float derivativeConstant,
								SystemControllerDirection controllerDirection) : SystemController()
{
	m_IntegrativeTermSum = 0;
	setSampleTime(sampleTime);
	setSetPoint(setPoint);
	setOutputLimits(minOutput, maxOutput);
	setTunings(proportionalConstant, integralConstant, derivativeConstant);
	setControllerDirection(controllerDirection);
}

void PIDController::setOutputLimits(float min, float max)
{
  SystemController::setOutputLimits(min, max);
  m_IntegrativeTermSum = constrain(m_IntegrativeTermSum, m_MinOutput, m_MaxOutput);
}

void PIDController::setTunings(float proportionalConstant, float integralConstant, float derivativeConstant)
{
	if (proportionalConstant < 0 || integralConstant < 0 || derivativeConstant < 0)
	{
		proportionalConstant = abs(proportionalConstant);
		integralConstant = abs(integralConstant);
		derivativeConstant = abs(derivativeConstant);
#ifdef DEBUG
		Serial.println("[PIDController::setTunings]: As constantes devem ser valores apenas positivos.");
		Serial.println("\tAs constantes foram modificadas para o valor absoluto delas. Use o modo de operação inverso.");
#endif
	}

	m_ProportionalConstant = proportionalConstant;

	/*
	Essa converção não é necessária, mas permite que a gente entre com
	valores de KI e KD em termos de Hz (1/s)
	*/
	/*
	integralConstant /= 1000;
	derivativeConstant /= 1000;
	*/

	m_IntegralConstant = integralConstant;
	m_DerivativeConstant = derivativeConstant;

	if(m_ControllerDirection == SystemControllerDirection::Inverse)
	{
		m_ProportionalConstant = -m_ProportionalConstant;
		m_IntegralConstant = -m_IntegralConstant;
		m_DerivativeConstant = -m_DerivativeConstant;
	}
}

float PIDController::run(float input)
{
	unsigned long deltaTime;
	if (!checkTime(&deltaTime))
	{
		return m_LastOutput;
	}
	return compute(input, deltaTime, m_ProportionalConstant, m_IntegralConstant, m_DerivativeConstant);
}

float PIDController::compute(float input, unsigned long deltaTime, float proportionalConstant, float integralConstant, float derivativeConstant)
{
	//Serial.print("compute");
	float error = m_SetPoint - input;

	// Se o tempo de amostragem for fixo, faz deltaTime = m_SampleTime
	deltaTime = (m_SampleTime != 0) ? m_SampleTime : deltaTime;

	/*
		Faz a derivada das entradas para evitar o "derivative kick", que ocorre
	mudando 	o setPoint. Esse fenomeno ocorre pois mudar o valor do setPoint
	causa uma 	variação instantânea no erro, e a derivada parcial desse erro em
	relação ao tempo é "infinito" (na prática dt não é zero então só acaba sendo
	um valor muito muito grande). Esse número entra no PID causando um pico
	indesejável no sinal de saída.
		(Trocar o setPoint não acontece em nenhum dos nossos projetos, mas é uma
	implementação melhor e o custo computacional é identico ao do PID clássico).

	Solução:
	 				d E(t) / dt = (d SP(t) / dt) - (d Input(t)/dt)

	Considere que o setPoint é uma função constante em relação ao tempo, logo:
	 											d SP(t) / dt = 0

	Então a derivada do erro é igual a menos a derivada do sinal de entrada:
	*/
	float dError = -(input - m_LastInput) / deltaTime;
	//float dError = (error - m_LastError) / deltaTime;


	/*
		Salva o valor acumulado do fator integrativo
		Isso torna possivel mudar a constante integrativa sem gerar uma mudança
	brusca na saída, já que o coeficinte que multiplicava o acumulo do erro já
	não é mais o mesmo. Se o Ki muda, então não podemos deixa-lo fora da integral
	como descrito dos algoritmos de PID clássicos logo nossa equação para o termo
	integrativo fica:

	 							Integral [t=0, t=m_Now] { Ki(t) * e(t) * dt }

		Na verdade essa equação é o "jeito certo" de se fazer PID, mas considerando Ki constante,
	a constante saí da integral e simplifica o calculo.

	Isso implementado resulta em:
	 				m_IntegrativeTermSum += error * integralConstant * deltaTime;

	Usando a aproximação trapezoidal para a integral fica:
	(Esse é um calculo um pouco mais custoso, faz uma discretização mais correta do tempo pro PID
	Existem outras aproximações que podemos explorar, mas não sei se é realmente necessário.
	Acho que na verdade esse tipo de aproximação não chega a fazer uma diferença real
	nos nossos casos de uso.)
	*/
	m_IntegrativeTermSum += deltaTime * integralConstant * (m_LastError + error) / 2.0;


	float output = proportionalConstant * error;	// Proporcional
	output += m_IntegrativeTermSum; 							// Integrativo
	output += derivativeConstant * dError; 				// Derivativo


	/*
	Restrige o erro acumulado pra evitar que este aumente indefinidamente
	e extrapole os limites que de trabalho do nosso sistema.
	Apesar da saída do sistema também ser limitado, a restrição do
	acumulo é necessária para que o sistema responda imediatamente a uma
	mudança na entrada e não tente compensar o integrativo desnecessariamente.
	Limita também o valor de saída, os fatores proporcional e derivativo também
	podem podem fazer com que a saída extrapole o intervalo de trabalho do sistema

	TL;DR: Isso evita o fenômeno conhecida como Reset Windup,
	que acontece quando o PID pensa que pode fazer algo que na verdade ele não pode
	provocando sobressinal que não é interpretado corretamente pelo sistema.

	TL;DR 2: Tira o lag da resposta do controle...
	*/
	if (output > m_MaxOutput)
	{
		m_IntegrativeTermSum -= output - m_MaxOutput;
		output = m_MaxOutput;
	}
	else if (output < m_MinOutput)
	{
		m_IntegrativeTermSum += m_MinOutput - output;
		output = m_MinOutput;
	}

	// Salva os valores pra proxima execução
	m_LastOutput = output;
	m_LastRunTime = m_Now;
	m_LastInput = input;
	m_LastError = error;
	return m_LastOutput;
}
