#include "pins.h"

#define MINIMUM_SPEED 0
#define ENTRE_EIXOS 1
#define RAIO_RODAS 10
#define MAXIMUM_SPEED 65 // 65

int leftPWM = 0;
int rightPWM  = 0;
float angularSpeed = 0;


void motorController(float linear, float angular);
void readRight();
void readLeft();

// velocidade linear controlada manualmente (?), velocidade angular controlada pelo PID)
void motorController(float angular) {
  float linear =  1 - abs(angular); // 1.5
  float leftSpeed = (linear + angular); // 1.0
  float rightSpeed  = (linear - angular); // 

  int leftPWM = map(leftSpeed, -1, 1, -MAXIMUM_SPEED, MAXIMUM_SPEED);//s + 1.55*s * leftSpeed;
  int rightPWM = map(rightSpeed, -1, 1, -MAXIMUM_SPEED, MAXIMUM_SPEED);//s + 1.55*s * rightSpeed;

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
void motorController2(float angular) {
  float leftSpeed = (+angular);
  float rightSpeed  = (-angular);

  int leftPWM = 40 + 127 * leftSpeed;
  int rightPWM = 40 + 127 * rightSpeed;

  rightPWM = constrain(rightPWM, 0, 150);
  leftPWM = constrain(leftPWM, 0, 150);
  Serial.print("Velocidade da direita:");
  Serial.print("\t");
  Serial.println("Velocidade da esquerda:");
  Serial.print(rightPWM);
  Serial.print("\t");
  Serial.println(leftPWM);

  analogWrite(L_MOTOR_1, leftPWM);
  analogWrite(L_MOTOR_2, 0);
  analogWrite(R_MOTOR_1, 0);
  analogWrite(R_MOTOR_2, rightPWM);
}


// a ideia dessa função é controlar a velocidade entre seu máximo e mínimo
