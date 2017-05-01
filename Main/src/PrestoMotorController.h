#ifndef PrestoMotorController_h
#define PrestoMotorController_h

#include "DifferentialDriveController.h"

class PrestoMotorController : public DifferentialDriveController
{
public:
  bool inCurve;
  PrestoMotorController();
  PrestoMotorController(int leftPin1, int leftPin2, int rightPin1, int rightPin2, float maxVelocity, float inCurveVelocity,
    float wheelsRadius, float wheelsDistance, WheelEncoder *pWheelEncoder);

  void setPins(int leftPin1, int leftPin2, int rightPin1, int rightPin2);
  void setVelocities(float maxVelocity, float inCurveVelocity);
  void move(float linearVelocity, float angularVelocity);

private:
  int _leftPin1;
  int _leftPin2;
  int _rightPin1;
  int _rightPin2;

  float _maxVelocity;
  float _inCurveVelocity;
};

#endif
