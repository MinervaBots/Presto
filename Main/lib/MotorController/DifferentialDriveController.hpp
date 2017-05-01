#ifndef DifferentialDriveController_hpp
#define DifferentialDriveController_hpp

#include "MotorController.hpp"
#include "../WheelEncoder/WheelEncoder.hpp"
#include "../Position/Position.hpp"

class DifferentialDriveController : public MotorController
{
public:
  DifferentialDriveController();
  DifferentialDriveController(Position* pPosition, float wheelsRadius, float wheelsDistance, WheelEncoder *pWheelEncoder);
  virtual void move(float linearVelocity, float angularVelocity);
  virtual void update();

  float getWheelsRadius() { return m_WheelsRadius; }
  float getWheelsDistance() { return m_WheelsDistance; }

  float getLeftVelocity() { return m_LeftVelocity; }
  float getRightVelocity() { return m_RightVelocity; }


  void setWheelsRadius(float wheelsRadius) { m_WheelsRadius = wheelsRadius; }
  void setWheelsDistance(float wheelsDistance) { m_WheelsDistance = wheelsDistance; }
  void setEncoder(WheelEncoder *pWheelEncoder) { m_pWheelEncoder = pWheelEncoder; }
  void setPositionPointer(Position *pPosition);

private:
  float m_WheelsRadius;
  float m_WheelsDistance;
  WheelEncoder *m_pWheelEncoder;
  float m_LeftVelocity;
  float m_RightVelocity;

  Position *m_pPosition;
};

#endif //DifferentialDriveController_hpp
