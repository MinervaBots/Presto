#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

void move(float angular, unsigned char maxPwm);
void stop();
void spin(float angular, unsigned char maxPwm);

#endif // MOTOR_CONTROL_H
