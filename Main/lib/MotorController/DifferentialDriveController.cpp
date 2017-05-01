#include "MotorController.h"
#include "DifferentialDriveController.h"

DifferentialDriveController::DifferentialDriveController()
{

}

DifferentialDriveController::DifferentialDriveController(float wheelsRadius, float wheelsDistance, WheelEncoder *pWheelEncoder) :
  _wheelsRadius(wheelsRadius),
  _wheelsDistance(wheelsDistance),
  _pWheelEncoder(pWheelEncoder)
{

}

void DifferentialDriveController::update()
{
  _pWheelEncoder->update();
}

void DifferentialDriveController::move(float linearVelocity, float angularVelocity)
{
  _leftVelocity = ((2 * linearVelocity) + (angularVelocity * _wheelsDistance)) / (2 * _wheelsRadius);
  _rightVelocity = ((2 * linearVelocity) - (angularVelocity * _wheelsDistance)) / (2 * _wheelsRadius);

  // Baseado no modelo
  //_heading += (_wheelsRadius / _wheelsDistance) * (_rightVelocity - _leftVelocity);

  // Usando o encoder
  _heading = _heading + ((_pWheelEncoder->getDeltaDistanceRight() - _pWheelEncoder->getDeltaDistanceLeft()) / _wheelsDistance);
}
