#ifndef CONSTANTS_H
#define CONSTANTS_H

#define BAUD_RATE 9600

#define NUM_SENSORS 11
#define ARRAY_CENTER_POSITION (((NUM_SENSORS - 1) / 2) * 1000)
#define LATERALS_CENTER_POSITION (((2 - 1) / 2) * 1000)
#define USE_WHITE_LINE 1
#define LINE_VALUE 300

#define MAX_BATTERY 970.0

#define STARTUP_DELAY 300
#define DEBOUNCE_TIME 500

#endif // CONSTANTS_H
