#ifndef SENSORS_HELPER_H
#define SENSORS_HELPER_H

void calibrateSensors();
void resetCalibration();

float readArray(bool *on_line);
bool readRight();

#endif // SENSORS_HELPER_H
