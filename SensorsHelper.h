#ifndef SENSORS_HELPER_H
#define SENSORS_HELPER_H

void calibrateSensors();

float readArray();

bool readRight(unsigned long maxCount);
void resetRightCount();

bool readLeft();

#endif // SENSORS_HELPER_H
