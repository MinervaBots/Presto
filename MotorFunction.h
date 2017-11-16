#include "pins.h"

#define MINIMUM_SPEED 0
#define ENTRE_EIXOS 1
#define RAIO_RODAS 10
#define MAXIMUM_SPEED 255

int leftPWM = 0;
int rightPWM  = 0;
float angularSpeed = 0;


void motorController(float linear,float angular);
void readRight();
void readLeft();

// velocidade linear controlada manualmente (?), velocidade angular controlada pelo PID)
void motorController(float linear,float angular){
   float rightSpeed = (linear + angular);
   float leftSpeed  = (linear - angular);

   int leftPWM = 127 + 127*leftSpeed;
   int rightPWM = 127 + 127*rightSpeed;
  
  rightPWM = constrain(rightPWM, 0, 150);
  leftPWM = constrain(leftPWM, 0, 150);
  Serial.print("Velocidade da direita:");
  Serial.print("\t");
  Serial.println("Velocidade da esquerda:");
  Serial.print(rightPWM);
  Serial.print("\t");
  Serial.println(leftPWM);

  analogWrite(L_MOTOR_1,leftPWM);
  analogWrite(L_MOTOR_2,0);
  analogWrite(R_MOTOR_1,0);
  analogWrite(R_MOTOR_2,rightPWM);
}


// a ideia dessa função é controlar a velocidade entre seu máximo e mínimo
