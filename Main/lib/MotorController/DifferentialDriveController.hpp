#ifndef DifferentialDriveController_hpp
#define DifferentialDriveController_hpp

#include "MotorController.hpp"
#include "../Position/Position.hpp"

class DifferentialDriveController : public MotorController
{
public:
  DifferentialDriveController() : DifferentialDriveController(0, 0) {}
  DifferentialDriveController(float wheelsRadius, float wheelsDistance);

  virtual void move(float linearVelocity, float angularVelocity);

  float getWheelsRadius() { return m_WheelsRadius; }
  float getWheelsDistance() { return m_WheelsDistance; }

  float getLeftVelocity() { return m_LeftVelocity; }
  float getRightVelocity() { return m_RightVelocity; }

  void setWheelsRadius(float wheelsRadius);
  void setWheelsDistance(float wheelsDistance);

protected:
  float m_LeftVelocity;
  float m_RightVelocity;

private:
  float m_WheelsRadius;
  float m_WheelsDistance;
};

#endif //DifferentialDriveController_hpp
