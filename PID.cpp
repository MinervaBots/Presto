#include "PID.h"
#include <Arduino.h>

PID::PID()
{

}

void PID::setOutputLimits(float minOutput, float maxOutput)
{
  _minOutput = minOutput;
  _maxOutput = maxOutput;

  _lastOutput = constrain(_lastOutput, _minOutput, _maxOutput);
  _integrativeSum = constrain(_integrativeSum, _minOutput, _maxOutput);
}

void PID::setTunings(float kP, float kI, float kD)
{
  _kP = kP;
  _kI = kI;
  _kD = kD;
}

void PID::setSampleTime(unsigned long newSampleTime)
{
  if (newSampleTime > 0)
  {
    _sampleTime = newSampleTime;
  }
}

void PID::setSetPoint(float setPoint)
{
  _setPoint = setPoint;
}

float PID::compute(float input)
{
  unsigned long now = millis();
  unsigned long timeChange = (now - _lastRunTime);
  if (timeChange < _sampleTime)
  {
    return _lastOutput;
  }

  float error = _setPoint - input;

  // Salva o valor acumulado do fator integrativo
  // Isso torna possivel mudar a constante integrativa sem gerar uma mudança busca
  // na saída, já que o coeficinte que multiplicava o acumulo do erro já não é mais o mesmo.
  // Se o Ki muda, então não podemos deixa-lo fora da integral como descrito dos algoritmos de
  // PID clássicos logo nossa equação para o termo integrativo fica:
  //
  //            Integral [t0 = 0, tf = now] { Ki(t) * e(t) * dt }
  //
  // Na verdade essa equação é o "jeito certo" de se fazer PID, mas considerando Ki constante,
  // a constante saí da integral e simplifica o calculo.
  //
  // Com a otimização de ter a variação do tempo multiplicado a constante de integração,
  // temos que a integral resulta no erro multiplicado apenas pela constante de integração:
  _integrativeSum += error * _kI * _sampleTime;
  // Limita a integração para evitar que ele continue integrando além do permitido pelo sistema
  // Ex: PWM (0 ~ 255)
  _integrativeSum = constrain(_integrativeSum, _minOutput, _maxOutput);

  // Faz a derivada da entrada para evitar o "derivative kick", que ocorre mudando
  // o setPoint. Esse fenomeno ocorre pois mudar o valor do setPoint causa uma
  // variação instantânea no erro, e a derivada parcial desse erro em relação
  // ao tempo é "infinito" (na prática dt não é zero então só acaba sendo um valor muito muito grande).
  // Esse número entra no PID causando um pico indesejável no sinal de saída.
  //
  // Solução:
  //        d(E(t)) / dt = (d(SetPoint(t)) / dt) - (d(Input(t)) / dt)
  //
  // Considere que o setPoint é uma função constante em relação ao tempo, logo:
  //                      d SP(t) / dt = 0
  //
  // Como a constante de derivação já aplica a variação do tempo, multiplicamos ela pela diferença
  // da entrada atual com a ultima entrada, e então a derivada do erro é simplesmente igual a
  // menos a derivada do sinal de entrada:
  float dInput = (_kD * (input - _lastInput)) / _sampleTime;
  float dError = -dInput;

  // Calcula a saída do PID e salva numa váriavel
  _lastOutput = (_kP * error) + (_integrativeSum) + (dError);
  // Limita a saída de acordo com o valor definido pelo usuário.
  // Cada sistema tem um valor de saída minimo e máximo.
  _lastOutput = constrain(_lastOutput, _minOutput, _maxOutput);

  _lastInput = input;
  _lastRunTime = now;

  return _lastOutput;
}


