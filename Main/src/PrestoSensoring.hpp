#ifndef PrestoSensoring_hpp
#define PrestoSensoring_hpp

#include "../lib/InputSource/InputSource.hpp"
#include "../lib/QTRSensors/QTRSensors.h"
#include "../lib/Button/Button.h"
#include "../lib/Filter/Filter.hpp"

class PrestoSensoring : public InputSource
{
public:
  PrestoSensoring();
  PrestoSensoring(QTRSensorsRC qtrArray, QTRSensorsRC qtrLeft, QTRSensorsRC qtrRight,
    unsigned long leftSampleTime, unsigned long rightSampleTime, Filter *pFilter);

  void setSensorArray(QTRSensorsRC qtrArray);
  void setSensorLeft(QTRSensorsRC qtrLeft) { m_QtrLeft = qtrLeft; }
  void setSensorRight(QTRSensorsRC qtrRight) { m_QtrRight = qtrRight; }
  void setSampleTimes(unsigned long leftSampleTime, unsigned long rightSampleTime);
  void setFilter(Filter *pFilter) { m_pFilter = pFilter; }
  unsigned int *getSensorWeights() { return m_SensorWeights; }

  void calibrate(Button commandButton, unsigned char statusLedPin);

  void update();
  bool shouldStop() { return m_RightCount >= 2; }
  bool inCurve() { return m_InCurve; }

  float getInput();

private:
  bool m_InCurve;
  unsigned long m_LeftSampleTime;
  unsigned int m_LeftSensorThreshold;

  unsigned long m_RightSampleTime;
  unsigned int m_RightSensorThreshold;

  unsigned int *m_SensorWeights;
  unsigned long m_LastRun;
  unsigned int m_RightCount;
  float m_CenterPosition;

  QTRSensorsRC m_QtrArray;
  QTRSensorsRC m_QtrLeft;
  QTRSensorsRC m_QtrRight;

  Filter *m_pFilter;
};

#endif // PrestoSensoring_hpp
