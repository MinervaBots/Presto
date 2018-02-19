#include <Arduino.h>
#include "inputs.h"
#include "outputs.h"
#include "controller.h"
#include "settings.h"

#ifndef STATES_H
#define STATES_H

void wait() {
	stop(); // outputs.h
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
	float kp = KP_SAFE;
	float ki = KD_SAFE;
	float kd = KD_SAFE;
	float offset = 0;
	float speed = SPEED_SAFE;
	while (stop > 0) {      
		if (leftMark()) { // inputs.h
	    	position++;
		}
	    if (position >= track_size) {
	    	kp = KP_SAFE;
			ki = KD_SAFE;
			kd = KD_SAFE;
			offset = 0;
			speed = SPEED_SAFE;
	    }
	    else {
	    	if (track[position] == 0) { // straight
	        	kp = KP_STRAIGHT;
	        	ki = KI_STRAIGHT;
	        	kd = KD_STRAIGHT;
	        	speed = SPEED_STRAIGHT;
	      	}
	      	else {
	        	kp = KP_TURN;
	        	ki = KI_TURN;
	        	kd = KD_TURN;
	        	speed = SPEED_TURN;
	        	if (track[position] == 1) { // right
	          		offset = OFFSET_TURN;
	        	}
	        	else { // left
	        		offset = -OFFSET_TURN;
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
