#ifndef PrestoSensoring_hpp
#define PrestoSensoring_hpp

//#include "../lib/CompilerDefinitions.h"
#include "../lib/InputSource/InputSource.hpp"
#include "../lib/Button/Button.h"

enum LineColor
{
  Black,
  White
};

class PrestoSensoring : public InputSource
{
public:
  PrestoSensoring();
  void setLineColor(LineColor lineColor) { m_LineColor = lineColor; }
  void setSensorArray(unsigned char* arrayPins, unsigned char pinsCount, unsigned long timeout);
  void setLeftSensor(unsigned char sensorPin, unsigned long sampleTime, unsigned long timeout);
  void setRightSensor(unsigned char sensorPin, unsigned long sampleTime, unsigned long timeout);

  void calibrate(Button commandButton, unsigned char statusLedPin);

  void update();
  bool shouldStop(unsigned int rightMarks);
  bool inCurve() { return m_InCurve; }

  float getInput();

private:
  LineColor m_LineColor;

  bool m_InCurve;
  unsigned char m_LeftSensorPin;
  unsigned long m_LeftSampleTime;
  unsigned int m_LeftMinReading;
  unsigned int m_LeftMaxReading;
  unsigned long m_LeftSensorTimeout;


  unsigned char m_RightSensorPin;
  unsigned long m_RightSampleTime;
  unsigned int m_RightMinReading;
  unsigned int m_RightMaxReading;
  unsigned long m_RightSensorTimeout;
  unsigned int m_RightCount;

  unsigned char m_SensorArrayCount;
  unsigned char *m_SensorArrayPins;
  unsigned int *m_SensorArrayMinReading;
  unsigned int *m_SensorArrayMaxReading;
  unsigned long m_SensorArrayTimeout;
  float m_CenterPosition;
  float m_LastValue;


  unsigned long m_LastRun;


  static unsigned int readPin(unsigned char pin, unsigned long timeout);
  static unsigned int clamp(unsigned int value, unsigned int min, unsigned int max);
};

#endif // PrestoSensoring_hpp
