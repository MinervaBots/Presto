#include "pins.h"

#define MINIMUM_SPEED 0
#define ENTRE_EIXOS 1
#define RAIO_RODAS 10
#define MAXIMUM_SPEED 130

void move(float linear, float angular)
{
  float leftSpeed = (linear + angular);
  float rightSpeed  = (linear - angular);

  int leftPWM = map(leftSpeed, -1, 1, -MAXIMUM_SPEED, MAXIMUM_SPEED);
  int rightPWM = map(rightSpeed, -1, 1, -MAXIMUM_SPEED, MAXIMUM_SPEED);

  rightPWM = abs(constrain(rightPWM, -MAXIMUM_SPEED, MAXIMUM_SPEED));
  leftPWM = abs(constrain(leftPWM, -MAXIMUM_SPEED, MAXIMUM_SPEED));
/*
  Serial.print("Velocidade da direita:");
  Serial.print("\t");
  Serial.println("Velocidade da esquerda:");
  Serial.print(rightPWM);
  Serial.print("\t");
  Serial.println(leftPWM);
*/
  if(leftSpeed > 0)
  {
    analogWrite(L_MOTOR_1, leftPWM);
    analogWrite(L_MOTOR_2, 0);
  }
  else
  {
    analogWrite(L_MOTOR_1, 0);
    analogWrite(L_MOTOR_2, leftPWM);
  }

  if(rightSpeed > 0)
  {
    analogWrite(R_MOTOR_1, 0);
    analogWrite(R_MOTOR_2, rightPWM);
  }
  else
  {
    analogWrite(R_MOTOR_1, rightPWM);
    analogWrite(R_MOTOR_2, 0);
  }
}
// velocidade linear controlada manualmente (?), velocidade angular controlada pelo PID)
void motorController(float angular) {
  float linear =  1 - abs(angular);
  move(linear, angular);
}

void stop()
{
  move(0, 0);
}

