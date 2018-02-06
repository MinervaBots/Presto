#include <Arduino.h>
#include "settings.h"
#include "inputs.h"
#include "outputs.h"


#ifndef CONTROLLER_H
#define CONTROLLER_H

void control(float kp,float ki,float kd,float speed,float offset); // implement

#endif
