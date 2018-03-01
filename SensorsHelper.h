#ifndef SENSORS_HELPER_H
#define SENSORS_HELPER_H

void calibrateSensors();

float readArray();
//void readLaterals(bool *left, bool* right);

bool readRight();
bool readLeft();

#endif // SENSORS_HELPER_H
