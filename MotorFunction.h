#include "pins.h"

#define MINIMUM_SPEED 0
#define ENTRE_EIXOS 1
#define RAIO_RODAS 2
#define MAXIMUM_SPEED 255

int leftPWM = 0;
int rightPWM  = 0;
float angularSpeed = 0;
unsigned rightValue = 0.0;
unsigned leftValue  = 0.0;

void motorController(float linear,float angular);
void readRight();
void readLeft();

// velocidade linear controlada manualmente (?), velocidade angular controlada pelo PID)
void motorController(float linear,float angular){
   float rightSpeed = ENTRE_EIXOS*(linear + angular)/(2*RAIO_RODAS);
   float leftSpeed  = ENTRE_EIXOS*(linear - angular)/(2*RAIO_RODAS);

   int leftPWM = 127 + 127*leftSpeed;
   int rightPWM = 127 + 127*rightSpeed;
     
  #ifdef DEBUG
    Serial.println("Velocidade da direita:/tVelocidade da esquerda:");
    Serial.print(rightSpeed);
    Serial.print("/t");
    Serial.println(leftSpeed);
  #endif
  
  rightPWM = constrain(rightPWM, 0, 255);
  leftPWM = constrain(leftPWM, 0, 255);
  
  analogWrite(L_MOTOR_1,leftPWM);
  analogWrite(L_MOTOR_2,0);
  analogWrite(R_MOTOR_1,rightPWM);
  analogWrite(R_MOTOR_2,0);
}


// a ideia dessa função é controlar a velocidade entre seu máximo e mínimo
