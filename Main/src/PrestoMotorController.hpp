#ifndef PrestoMotorController_hpp
#define PrestoMotorController_hpp

#include "../lib/MotorController/DifferentialDriveController.hpp"

class PrestoMotorController : public DifferentialDriveController
{
public:
  PrestoMotorController() : PrestoMotorController(255, 255, 255, 255) {}
  PrestoMotorController(unsigned char leftPin1, unsigned char leftPin2,
    unsigned char rightPin1, unsigned char rightPin2);

  void stop();
  void update();
  void move(float linearVelocity, float angularVelocity);

  void setPins(unsigned char leftPwmPin, unsigned char leftDirectionPin,
    unsigned char rightPwmPin, unsigned char rightDirectionPin);

  bool IsLeftForward() { return m_IsLeftForward; }

private:
  bool m_IsLeftForward;
  unsigned char m_LeftPin1;
  unsigned char m_LeftPin2;

  unsigned char m_RightPin1;
  unsigned char m_RightPin2;
};

#endif // PrestoMotorController_hpp
