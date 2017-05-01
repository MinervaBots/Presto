#ifndef PrestoMotorController_hpp
#define PrestoMotorController_hpp

#include "../lib/MotorController/DifferentialDriveController.hpp"

class PrestoMotorController : public DifferentialDriveController
{
public:
  bool inCurve;
  PrestoMotorController();
  PrestoMotorController(unsigned int leftPwmPin, unsigned int leftInPin1, unsigned int leftInPin2,
    unsigned int rightPwmPin, unsigned int rightInPin1, unsigned int rightInPin2, unsigned int standbyPin,
    float maxVelocity, float inCurveVelocity,
    float wheelsRadius, float wheelsDistance, WheelEncoder *pWheelEncoder);

  void setPins(unsigned int leftPwmPin, unsigned int leftInPin1, unsigned int leftInPin2,
    unsigned int rightPwmPin, unsigned int rightInPin1, unsigned int rightInPin2, unsigned int standbyPin);
  void setVelocities(float maxVelocity, float inCurveVelocity);

  void stop();
  void move(float linearVelocity, float angularVelocity);

private:
  unsigned int m_LeftPwmPin;
  unsigned int m_LeftInPin1;
  unsigned int m_LeftInPin2;

  unsigned int m_RightPwmPin;
  unsigned int m_RightInPin1;
  unsigned int m_RightInPin2;

  unsigned int m_StandbyPin;

  float m_MaxVelocity;
  float m_InCurveVelocity;
};

#endif // PrestoMotorController_hpp
