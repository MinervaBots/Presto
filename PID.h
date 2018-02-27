#ifndef PID_H
#define PID_H

class PID
{
public:
  PID();

  void setOutputLimits(float minOutput, float maxOutput);
  void setTunings(float kP, float kI, float kD);
  
  void setSampleTime(unsigned long newSampleTime);
  void setSetPoint(float setPoint);

  float getKP() { return _kP; }
  float getKI() { return _kI; }
  float getKD() { return _kD; }
   
  float compute(float input);

private:
  float _setPoint;
  float _minOutput, _maxOutput;

  float _integrativeSum;
  float _lastOutput;
  float _lastInput;
  float _kP, _kI, _kD;
  unsigned int _sampleTime, _lastRunTime;
  
};

#endif //PID_H
