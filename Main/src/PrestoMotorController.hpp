#ifndef PrestoMotorController_hpp
#define PrestoMotorController_hpp

#include "../lib/MotorController/DifferentialDriveController.hpp"

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

  void setMaxPWM(unsigned int maxPWM);
  void setActivationSmoothingValue(unsigned int smoothingValue);

protected:
  void setPins(unsigned char leftInPin1, unsigned char leftInPin2,
    unsigned char rightInPin1, unsigned char rightInPin2);

private:
  unsigned char m_LeftInPin1;
  unsigned char m_LeftInPin2;

  unsigned char m_RightInPin1;
  unsigned char m_RightInPin2;

  unsigned int m_MaxPWM;
  unsigned int m_SmoothingValue;

  /*
  Funções de ativação (https://en.wikipedia.org/wiki/Activation_function)
  Fiz umas modificações pra atender nossas necessidades.

  Elas não são exatamente pesadas, mas são mais custosas que um simples 'clamp',
  mas são resultados muito melhores, e acredito que com funções assim não vamos
  ter problema com reversão de motores.
  */
  static float softSign(float x, unsigned int b);
  static float hyperbolicTangent(float x, unsigned int b);
};

#endif // PrestoMotorController_hpp
