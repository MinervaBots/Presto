#ifndef PrestoMotorController_hpp
#define PrestoMotorController_hpp

#include "../lib/MotorController/DifferentialDriveController.hpp"

// TODO - Como calcular essas constantes?
#define TENSAO_DE_ALIMENTACAO 10.0
#define KM 7.8164

class PrestoMotorController : public DifferentialDriveController
{
public:
  bool inCurve;
  PrestoMotorController();
  PrestoMotorController(unsigned int leftInPin1, unsigned int leftInPin2,
    unsigned int rightInPin1, unsigned int rightInPin2,
    float wheelsRadius, float wheelsDistance, WheelEncoder *pWheelEncoder);

  void setPins(unsigned int leftInPin1, unsigned int leftInPin2,
    unsigned int rightInPin1, unsigned int rightInPin2);
  void setVelocities(float maxVelocity, float inCurveVelocity);

  void stop();
  void move(float linearVelocity, float angularVelocity);

private:
  unsigned int m_LeftInPin1;
  unsigned int m_LeftInPin2;

  unsigned int m_RightInPin1;
  unsigned int m_RightInPin2;

  unsigned int m_LeftPwm;
  unsigned int m_RightPwm;

  void calculatePwm();
};

#endif // PrestoMotorController_hpp
