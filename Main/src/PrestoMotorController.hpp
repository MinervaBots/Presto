#ifndef PrestoMotorController_hpp
#define PrestoMotorController_hpp

#include "../lib/MotorController/DifferentialDriveController.hpp"

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
  int m_LeftPin1;
  int m_LeftPin2;
  int m_RightPin1;
  int m_RightPin2;

  float m_MaxVelocity;
  float m_InCurveVelocity;
};

#endif // PrestoMotorController_hpp
