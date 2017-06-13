#ifndef PrestoMotorController_hpp
#define PrestoMotorController_hpp

#include "../lib/MotorController/DifferentialDriveController.hpp"

class PrestoMotorController : public DifferentialDriveController
{
public:
  PrestoMotorController() : PrestoMotorController(255, 255, 255, 255) {}
  PrestoMotorController(unsigned char leftPwmPin, unsigned char leftDirectionPin,
    unsigned char rightPwmPin, unsigned char rightDirectionPin);

  void stop();
  void update();
  void move(float linearVelocity, float angularVelocity);

  void setMaxPWM(unsigned int maxPWM);
  void setLinearVelocityRatio(float velocityRatio) { m_LinearVelocityRatio = velocityRatio; }

  void setPins(unsigned char leftPwmPin, unsigned char leftDirectionPin,
    unsigned char rightPwmPin, unsigned char rightDirectionPin);

private:
  unsigned char m_LeftPwmPin;
  unsigned char m_LeftDirectionPin;

  unsigned char m_RightPwmPin;
  unsigned char m_RightDirectionPin;

  unsigned int m_MaxPWM;
  float m_LinearVelocityRatio;
};

#endif // PrestoMotorController_hpp
