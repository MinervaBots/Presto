#include "PIDController.h"
#include <Arduino.h>

float clamp(float value, float min, float max)
{
	if (value < min)
		return min;
	else if (value > max)
		return max;
	return value;
}


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

PIDController::~PIDController()
{

}

void PIDController::setSampleTime(int newSampleTime)
{
  if (newSampleTime > 0 && _sampleTime > 0)
	{
		float ratio = newSampleTime / _sampleTime;
		_integralConstant *= ratio;
		_derivativeConstant /= ratio;
  }
  SystemController::setSampleTime(newSampleTime);
}

void PIDController::setOutputLimits(float min, float max)
{
  SystemController::setOutputLimits(min, max);
  _integrativeTermSum = clamp(_integrativeTermSum, _minOutput, _maxOutput);
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

	_proportionalConstant = proportionalConstant;
	// Essa converção não é necessária, mas permite que a gente entre com
	// valores
	// de KI e KD em termos de 1/segundo
	float sampleTimeInSec = ((float) _sampleTime) / 1000;
	_integralConstant = integralConstant * sampleTimeInSec;
	_derivativeConstant = derivativeConstant / sampleTimeInSec;
	// A aplicação direta dos valores aqui nas constantes só é possivel
	// porque o
	// tempo de avalição do PID é fixado. Isso evita também que a
	// multiplicação
	// E principalmente a divisão tenham que ser feitas cada vez que o PID é
	// calculado
	// tl;dr: deixa o código mais rápido e mais eficiente.

	if (_controllerDirection == SystemControllerDirection::Inverse)
	{
		_proportionalConstant *= -1;
		_integralConstant *= -1;
		_derivativeConstant *= -1;
	}
}

float PIDController::run(float input)
{
	long now = millis();
	unsigned long deltaTime = (now - _lastRunTime);
	if (deltaTime < _sampleTime)
	{
		return _lastOutput;
	}

	float error = _setPoint - input;

	// Salva o valor acumulado do fator integrativo
	// Isso torna possivel mudar a constante integrativa sem gerar uma
	// mudança abruta na saída
	// já que o acumulo dos erros não é mais multiplicado pelo mesmo valor
	// que antes
	_integrativeTermSum += error * _integralConstant;
	// Faz o clamp disso pra evitar que o erro se acumule indefinidamente]
	// e extrapole os limites que o nosso sistema usa.
	// Apesar da saída do sistema também ser limitado, precisa fazer do
	// acumulo dos erros
	// pra que o sistema responda imediatamente a uma mudança na entrada e
	// não tente compensar o integrativo desnecessariamente
	_integrativeTermSum = clamp(_integrativeTermSum, _minOutput, _maxOutput);

	// Faz a derivada das entradas para evitar o "derivative kick", que
	// ocorre mudando o setPoint
	// Não acontece em nenhum dos nossos projetos, mas é uma implementação
	// melhor,
	// e o custo computacional é identico
	float dInput = (input - _lastInput);

	float output = _proportionalConstant * error; // Proporcional
	output += _integrativeTermSum; // Integrativo
	output -= _derivativeConstant * dInput; // Derivativo

	// Faz clamp da saída do PID também, pois os fatores proporcional e
	// derivativo também
	// podem fazer com que a saída extrapole o intervalo de trabalho do
	// sistema
	output = clamp(output, _minOutput, _maxOutput);

	_lastInput = input;
	_lastRunTime = now;
	_lastOutput = output;

	return output;
}
