#include "PrestoMotorController.hpp"
#include "../lib/MathHelper/MathHelper.h"
#include <Arduino.h>

PrestoMotorController::PrestoMotorController(unsigned char leftInPin1, unsigned char leftInPin2,
  unsigned char rightInPin1, unsigned char rightInPin2,
  float wheelsRadius, float wheelsDistance, WheelEncoder *pWheelEncoder) :
  DifferentialDriveController(wheelsRadius, wheelsDistance, nullptr, pWheelEncoder)
{
  setPins(leftInPin1, leftInPin2, rightInPin1, rightInPin2);

  // Define um valor intermediário
  setActivationSmoothingValue(15);

  // Velocidade máxima não parece ser uma boa ideia
  setMaxPWM(200);
}

void PrestoMotorController::setPins(unsigned char leftInPin1, unsigned char leftInPin2,
  unsigned char rightInPin1, unsigned char rightInPin2)
{
  m_LeftInPin1 = leftInPin1;
  m_LeftInPin2 = leftInPin2;

  m_RightInPin1 = rightInPin1;
  m_RightInPin2 = rightInPin2;

  pinMode(m_LeftInPin1, OUTPUT);
  pinMode(m_LeftInPin2, OUTPUT);

  pinMode(m_RightInPin1, OUTPUT);
  pinMode(m_RightInPin2, OUTPUT);
}

void PrestoMotorController::stop()
{
  DifferentialDriveController::stop();
  analogWrite(m_LeftInPin1, 255);
  analogWrite(m_LeftInPin2, 255);
  analogWrite(m_RightInPin1, 255);
  analogWrite(m_RightInPin2, 255);
}

void PrestoMotorController::setMaxPWM(unsigned int maxPWM)
{
  m_MaxPWM = maxPWM;
  if(m_MaxPWM > 255)
  {
    m_MaxPWM = 255;
  }
}

void PrestoMotorController::setActivationSmoothingValue(unsigned int smoothingValue)
{
  m_SmoothingValue = smoothingValue;
  /*
  if(m_SmoothingValue > 20)
  {
    m_SmoothingValue = 20;
  }
  */
}

void PrestoMotorController::move(float linearVelocity, float angularVelocity)
{
  // Reduzia a velocidade linear em função da angular?
  //DifferentialDriveController::move(linearVelocity, angularVelocity);
  m_LeftVelocity = linearVelocity + angularVelocity;
  m_RightVelocity = linearVelocity - angularVelocity;

  /*
  Impede que as velocidades sejam negativas, evitando reversão dos motores.
  Caso seja negativa, ela assuma o valor 0, fazendo com que essa roda seja o
  eixo de rotação do Presto.
  */
  if(m_LeftVelocity < 0)
    m_LeftVelocity = 0;
  if(m_RightVelocity < 0)
    m_RightVelocity = 0;

  /*
  Define esses pinos como 0 forçando o movimento a ser sempre CC,
  resultando apenas em movimento para frente.
  */
  analogWrite(m_LeftInPin2, 0);
  analogWrite(m_RightInPin2, 0);

  /*
  Usa a uma função de ativação pra converter as velocidades em PWM.

  As funções que eu conheço que podemos usar são:
  -Softsign;
  -TanH;
  Todas elas tem um range de -1 a 1, com 0 resultado em 0.

  A TanH atinge os limites mais rapidamente que Softsign, que é mais suave, mas
  ambas tem o seu intervalo de variação relativamente curto, por isso introduzi
  um outro parâmetro 'b' para podermos controlar mais a suavidade de ambas as funções.
  'b' varia de 1 a 20 (na real o quanto você quiser, mas eu acredito que mais que
  isso é desnecessário, a curva vai ficando cada vez menos inclinada e demora muito
  pra chegar em -1, principalmente Softsign) e quanto maior o valor de 'b' mais suave é a função.
  *Esse valor depende muito do intervalo de saída do PID, e do modelo de controle do Presto.

  Podemos usar Sigmoid (ou Logistic, ou Soft step) também mas o range dela é de 0 a 1,
  então entrando 0 saí 0.5, que deve ser subtraido no final.
  Também é possivel usar arctan (tan^-1) também mas o range dela é de -pi/2 a pi/2,
  dividindo por pi/2 normalizamos o valor, mas não vejo porque, já que qualquer uma acima
  dá praticamente o mesmo resultado.

  Usando essas funções de ativação, nunca vamos ter um valor de PWM igual a 255,
  já que essas funções só retornam 1 quando o parâmetro é infinito, mas devido a
  aproximações podemos chegar bem perto (perto o suficiente pra ser arredondo pra 255).
  E as vezes nem é a nossa intenção ir tão rápido então é só mudar essa constante.
  */
  int leftPwm = m_MaxPWM * hyperbolicTangent(m_LeftVelocity, m_SmoothingValue);
  int rightPwm = m_MaxPWM * hyperbolicTangent(m_RightVelocity, m_SmoothingValue);

  analogWrite(m_LeftInPin1, leftPwm);
	analogWrite(m_RightInPin1, rightPwm);

/*
  Serial.print("L VEL: ");
  Serial.println(m_LeftVelocity);
  Serial.print("R VEL: ");
  Serial.println(m_RightVelocity);

  Serial.print("L PWM: ");
  Serial.println(leftPwm);
  Serial.print("R PWM: ");
  Serial.println(rightPwm);
*/
}

//[TODO] - Provavelmente tá errado
float PrestoMotorController::softSign(float x, unsigned int b)
{
  return x / ((10 / ((float)b)) + x);
}

float PrestoMotorController::hyperbolicTangent(float x, unsigned int b)
{
  return (2 / (1 + exp((-2 * (1/(float)b) * x)))) - 1;
}
