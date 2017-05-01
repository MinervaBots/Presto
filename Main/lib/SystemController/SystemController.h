#ifndef SystemController_h
#define SystemController_h

enum SystemControllerDirection
{
  Inverse = -1,
  Direct = 1
};

class SystemController
{
public:
  virtual float run(float intput) = 0;

  virtual void setSetPoint(float newSetPoint);
  virtual void setSampleTime(unsigned long newSampleTime);
  virtual void setOutputLimits(float min, float max);
  virtual void setControllerDirection(SystemControllerDirection direction);

protected:
  unsigned long _sampleTime;
  unsigned long _lastRunTime;
  float _setPoint;
  float _lastOutput;
  float _lastInput;
  float _minOutput;
  float _maxOutput;

  SystemControllerDirection _controllerDirection;
};

#endif // SystemController_h
