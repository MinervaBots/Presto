#include "QTRSensors.h"

#define COLOR_THRESHOLD 500 // Values greater then this are considered dark, otherwise they are considered light

const int encoder_pin = 10;
const int ATmega_pin = PIN_B4; 
QTRSensorsRC encoder((unsigned char[]) { encoder_pin }, 1);

int output_level = 0; 
int colorChanged(int sensor_reading);

void setup()
{
  pinMode(encoder_pin, INPUT);
  pinMode(ATmega_pin, OUTPUT);
  digitalWrite(ATmega_pin, output_level);

  // pinMode(13, HIGH);

  for (int i = 0; i < 400; i++)
  {
    encoder.calibrate();
  }
  
  //digitalWrite(13, LOW);
}

void loop()
{
  unsigned int reading[1];
  encoder.readCalibrated(reading);
  
  if(colorChanged(reading[0])) {
    // trigger interrupt on ATmega
    digitalWrite(ATmega_pin, output_level ^= 1); // Toggle pin
  }
}

int colorChanged(int sensor_value) {
    static int last_color = sensor_value;

    if((sensor_value >= COLOR_THRESHOLD) != (last_color >= COLOR_THRESHOLD)) {
        last_color = sensor_value;
        return 1;
    }

    return 0;
}
