#ifndef SENSORS_HELPER_H
#define SENSORS_HELPER_H

void calibrateSensors();
void resetCalibration();

float readArray();
void readLaterals(bool *left, bool* right);

#endif // SENSORS_HELPER_H
