#ifndef PrestoSensoring_h
#define PrestoSensoring_h

#include "InputSource.h"
#include "QTRSensors.h"
#include "Button.h"

class PrestoSensoring : public InputSource
{
public:
  PrestoSensoring();
  PrestoSensoring(QTRSensorsRC qtrArray, QTRSensorsRC qtrLeft, QTRSensorsRC qtrRight,
    unsigned long leftSampleTime, unsigned long rightSampleTime, unsigned int *sensorWeights);

  void setSensorArray(QTRSensorsRC qtrArray);
  void setSensorLeft(QTRSensorsRC qtrLeft);
  void setSensorRight(QTRSensorsRC qtrRight);
  void setSampleTimes(unsigned long leftSampleTime, unsigned long rightSampleTime);
  void setSensorWeights(unsigned int *sensorWeights);

  void calibrate(Button commandButton, unsigned int statusLedPin);

  void update();
  bool shouldStop();
  bool inCurve();

  float getInput();

private:
  float erro_maximo = 10.0;
  bool _inCurve;
  unsigned long _leftSampleTime;
  unsigned long _rightSampleTime;
  unsigned int *_sensorWeights;
  unsigned long _lastRun;
  unsigned int _rightCount;

  QTRSensorsRC _qtrArray;
  QTRSensorsRC _qtrRight;
  QTRSensorsRC _qtrLeft;
};

#endif // PrestoSensoring_h
