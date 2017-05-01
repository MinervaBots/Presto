#ifndef DifferentialDriveController_h
#define DifferentialDriveController_h

#include "MotorController.h"
#include "../WheelEncoder/WheelEncoder.h"

class DifferentialDriveController : public MotorController
{
public:
  DifferentialDriveController();
  DifferentialDriveController(float wheelsRadius, float wheelsDistance, WheelEncoder *pWheelEncoder);
  virtual void move(float linearVelocity, float angularVelocity);
  virtual void update();
  
  float getWheelsRadius() { return _wheelsRadius; }
  float getWheelsDistance() { return _wheelsDistance; }
  float getLeftVelocity() { return _leftVelocity; }
  float getRightVelocity() { return _rightVelocity; }
  float getHeading() { return _heading; }

  void setWheelsRadius(float wheelsRadius) { _wheelsRadius = wheelsRadius; }
  void setWheelsDistance(float wheelsDistance) { _wheelsDistance = wheelsDistance; }
  void setEncoder(WheelEncoder *pWheelEncoder) { _pWheelEncoder = pWheelEncoder; }

private:
  float _wheelsRadius;
  float _wheelsDistance;
  WheelEncoder *_pWheelEncoder;
  float _leftVelocity;
  float _rightVelocity;
  float _heading;
};

#endif //DifferentialDriveController_h
