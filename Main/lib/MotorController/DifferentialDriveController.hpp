#ifndef DifferentialDriveController_hpp
#define DifferentialDriveController_hpp

#include "MotorController.hpp"
#include "../WheelEncoder/WheelEncoder.hpp"
#include "../Position/Position.hpp"

class DifferentialDriveController : public MotorController
{
public:
  DifferentialDriveController()
      : DifferentialDriveController(0, 0, nullptr, nullptr) {}
  DifferentialDriveController(float wheelsRadius, float wheelsDistance)
      : DifferentialDriveController(wheelsRadius, wheelsDistance, nullptr, nullptr) {}
  DifferentialDriveController(float wheelsRadius, float wheelsDistance, Position* pPosition)
      : DifferentialDriveController(wheelsRadius, wheelsDistance, pPosition, nullptr) {}
  DifferentialDriveController(float wheelsRadius, float wheelsDistance, Position* pPosition, WheelEncoder *pWheelEncoder);
  virtual void move(float linearVelocity, float angularVelocity);
  virtual void update();
  virtual void updatePosition();

  float getWheelsRadius() { return m_WheelsRadius; }
  float getWheelsDistance() { return m_WheelsDistance; }

  float getLeftVelocity() { return m_LeftVelocity; }
  float getRightVelocity() { return m_RightVelocity; }


  void setWheelsRadius(float wheelsRadius) { m_WheelsRadius = wheelsRadius; }
  void setWheelsDistance(float wheelsDistance) { m_WheelsDistance = wheelsDistance; }
  void setEncoder(WheelEncoder *pWheelEncoder) { m_pWheelEncoder = pWheelEncoder; }
  void setPositionPointer(Position *pPosition) { m_pPosition = pPosition; }

private:
  float m_WheelsRadius;
  float m_WheelsDistance;
  WheelEncoder *m_pWheelEncoder;
  float m_LeftVelocity;
  float m_RightVelocity;

  Position *m_pPosition;
};

#endif //DifferentialDriveController_hpp
