#ifndef PrestoSensoring_hpp
#define PrestoSensoring_hpp

#include "InputSource.hpp"
#include "QTRSensors.h"
#include "Button.h"

class PrestoSensoring : public InputSource
{
public:
  PrestoSensoring();
  PrestoSensoring(QTRSensorsRC qtrArray, QTRSensorsRC qtrLeft, QTRSensorsRC qtrRight,
    unsigned long leftSampleTime, unsigned long rightSampleTime, unsigned int *sensorWeights);

  void setSensorArray(QTRSensorsRC qtrArray) { m_QtrArray = qtrArray; }
  void setSensorLeft(QTRSensorsRC qtrLeft) { m_QtrLeft = qtrLeft; }
  void setSensorRight(QTRSensorsRC qtrRight) { m_QtrRight = qtrRight; }
  void setSampleTimes(unsigned long leftSampleTime, unsigned long rightSampleTime);
  void setSensorWeights(unsigned int *sensorWeights) { m_SensorWeights = sensorWeights; }

  void calibrate(Button commandButton, unsigned int statusLedPin);

  void update();
  bool shouldStop() { return m_RightCount >= 2; }
  bool inCurve() { return m_InCurve; }

  float getInput();

private:
  float erro_maximo = 10.0;
  bool m_InCurve;
  unsigned long m_LeftSampleTime;
  unsigned long m_RightSampleTime;
  unsigned int *m_SensorWeights;
  unsigned long m_LastRun;
  unsigned int m_RightCount;

  QTRSensorsRC m_QtrArray;
  QTRSensorsRC m_QtrLeft;
  QTRSensorsRC m_QtrRight;
};

#endif // PrestoSensoring_hpp
