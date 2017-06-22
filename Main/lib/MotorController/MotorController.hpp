#ifndef MotorController_hpp
#define MotorController_hpp

#include "../CompilerDefinitions.h"

class MotorController
{
public:
  virtual void stop() { move(0, 0); }
  virtual void move(float linearSpeed, float angularSpeed) = 0;
};

#endif //MotorController_hpp
