#include <Arduino.h>

#ifndef OUTPUTS_H
#define OUTPUTS_H

void follow(float, float);
void returnToLine(bool);
void convertSignal(int, int);
void stop();

void follow(float linear, float angular) { // values from -1 to 1
	linear = constrain(linear, -1, 1);
	angular = constrain(angular, -1, 1);

	int left_speed, right_speed;
	if (angular > 0) {
		right_speed = constrain(1000*linear, -1000, 1000);
		left_speed = constrain(1000*(linear - angular), -1000, 1000);
	}
	else {
		left_speed = constrain(1000*linear, -1000, 1000);
		right_speed = constrain(1000*(linear - angular), -1000, 1000);
	}

	convertSignal(left_speed, right_speed);
}

void returnToLine(bool right) { // 0 = sharp turn to left ; 1 = sharp turn to right
	// use follow(0, -1 + right*2) ???
	if (right) {
		convertSignal(1000, 0); // verify speeds
	}
	else {
		convertSignal(0, 1000); // verify speeds
	}
}

void convertSignal(int left_speed, int right_speed) { // values from -1000 to 1000
	int left_pwm = map(left_speed, -1000, 1000, -255, 255);
	int right_pwm = map(right_speed, -1000, 1000, -255, 255);
	analogWrite(LEFT_MOTOR_PWM, abs(left_pwm));
	analogWrite(RIGHT_MOTOR_PWM, abs(right_pwm));

	if (left_speed > 0) {
		digitalWrite(LEFT_MOTOR_P1, 1); // verify driver datasheet
		digitalWrite(LEFT_MOTOR_P2, 0); // verify driver datasheet
	}
	else {
		digitalWrite(LEFT_MOTOR_P1, 0); // verify driver datasheet
		digitalWrite(LEFT_MOTOR_P2, 1); // verify driver datasheet
	}

	if (right_speed > 0) {
		digitalWrite(RIGHT_MOTOR_P1, 1); // verify driver datasheet
		digitalWrite(RIGHT_MOTOR_P2, 0); // verify driver datasheet
	}
	else {
		digitalWrite(RIGHT_MOTOR_P1, 0); // verify driver datasheet
		digitalWrite(RIGHT_MOTOR_P2, 1); // verify driver datasheet
	}
}

void stop() {
	follow(0, 0);
}

#endif
