#include <Arduino.h>
#include "inputs.h"
#include "outputs.h"
#include "controller.h"

#ifndef STATES_H
#define STATES_H

void wait() {
  while (1) {
    if (button.pressed()) { // inputs.h
      break;
    }
  }
}

void calibrate() {
  while (1) {

    ir_array.calibrate(); // inputs.h
    left_mark.calibrate(); // inputs.h
    right_mark.calibrate(); // inputs.h

    if (button.pressed()) { // inputs.h
      break;
    }
  }
}

void run(int track[], int track_size) {
  int stop = 2;
  position = 0;
  // safe constants
  float kp = 1;
  float ki = 0;
  float kd = 0;
  float offset = 0;
  float speed = 0.5;
  while (stop > 0) {      
    if (leftMark()) { // inputs.h
      position++;
    }
    if (position >= track_size) {
      safeRun();
    }
    else {
      if (track[position] == 0) { // straight
        kp = 1;
        ki = 0;
        kd = 0;
        speed = 0.5;
      }
      else {
        kp = 1;
        ki = 0;
        kd = 0;
        speed = 0.5;
        if (track[position] == 1) { // right
          offset = 1;
        }
        else { // left
          offset = -1;
        }
      }
    }
    if (rightMark()) { // inputs.h
      stop--;
    }
    control(kp,ki,kd,speed,offset); // controller.h
  }
  stop(); // outputs.h

}

#endif
