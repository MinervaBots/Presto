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
  PrestoMotorController(unsigned char leftInPin1, unsigned char leftInPin2,
    unsigned char rightInPin1, unsigned char rightInPin2) :
      PrestoMotorController(leftInPin1, leftInPin2, rightInPin1, rightInPin2, 0, 0, nullptr) {}

  PrestoMotorController(unsigned char leftInPin1, unsigned char leftInPin2,
    unsigned char rightInPin1, unsigned char rightInPin2,
    float wheelsRadius, float wheelsDistance, WheelEncoder *pWheelEncoder);

  void setVelocities(float maxVelocity, float inCurveVelocity);

  void stop();
  void move(float linearVelocity, float angularVelocity);

protected:
  void setPins(unsigned char leftInPin1, unsigned char leftInPin2,
    unsigned char rightInPin1, unsigned char rightInPin2);

private:
  unsigned char m_LeftInPin1;
  unsigned char m_LeftInPin2;

  unsigned char m_RightInPin1;
  unsigned char m_RightInPin2;

  unsigned int m_LeftPwm;
  unsigned int m_RightPwm;

  void calculatePwm();
};

#endif // PrestoMotorController_hpp
