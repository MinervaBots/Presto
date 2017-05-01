#ifndef MotorController_h
#define MotorController_h

class MotorController
{
public:
  virtual void move(float linearSpeed, float angularSpeed) = 0;
  virtual void update() = 0; // Atualiza encoders ou qualquer outra coisa necessária
};

#endif //MotorController_h
