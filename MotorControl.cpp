#include "MotorControl.h"
#include "Pins.h"
#include <Arduino.h>

void halfMove(float linear, float angular, unsigned char maxPwm)
{
  float leftSpeed = (linear + angular);
  float rightSpeed  = (linear - angular);

  int leftPWM = map(leftSpeed, 0, 1, 0, maxPwm);
  int rightPWM = map(rightSpeed, 0, 1, 0, maxPwm);

  rightPWM = constrain(rightPWM, 0, maxPwm);
  leftPWM = constrain(leftPWM, 0, maxPwm);

  analogWrite(L_MOTOR_1, leftPWM);
  analogWrite(L_MOTOR_2, 0);

  analogWrite(R_MOTOR_1, 0);
  analogWrite(R_MOTOR_2, rightPWM);
}

void fullMove(float linear, float angular, unsigned char maxPwm)
{
  float leftSpeed = (linear + angular);
  float rightSpeed  = (linear - angular);

  int leftPWM = map(leftSpeed, -1, 1, -maxPwm, maxPwm);
  int rightPWM = map(rightSpeed, -1, 1, -maxPwm, maxPwm);

  rightPWM = abs(constrain(rightPWM, -maxPwm, maxPwm));
  leftPWM = abs(constrain(leftPWM, -maxPwm, maxPwm));

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

void move(float angular, unsigned char maxPwm, bool half)
{
  float linear = 1 - abs(angular);
  if(half)
  {
    halfMove(linear, angular, maxPwm);
    return;
  }
  fullMove(linear, angular, maxPwm);
}

void spin(float angular, unsigned char maxPwm)
{
  fullMove(0, angular, maxPwm);
}

void stop()
{
  analogWrite(L_MOTOR_1, 0);
  analogWrite(L_MOTOR_2, 0);
  analogWrite(R_MOTOR_1, 0);
  analogWrite(R_MOTOR_2, 0);
}
