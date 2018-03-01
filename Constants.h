#ifndef CONSTANTS_H
#define CONSTANTS_H

#define BAUD_RATE 9600 //38400 é o padrão para do módulo bluetooth

#define NUM_SENSORS 8
#define ARRAY_CENTER_POSITION (((NUM_SENSORS - 1) / 2) * 1000)
#define LATERALS_CENTER_POSITION (((2 - 1) / 2) * 1000)
#define USE_WHITE_LINE 1
#define LINE_VALUE 500

#define STARTUP_DELAY 300
#define DEBOUNCE_TIME 500

#define RIGHT_SENSOR_LOW_TIME 400
#define LEFT_SENSOR_LOW_TIME 400

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#endif // CONSTANTS_H
