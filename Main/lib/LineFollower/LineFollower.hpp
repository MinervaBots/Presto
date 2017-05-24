#ifndef LineFollower_hpp
#define LineFollower_hpp

#include "../CompilerDefinitions.h"
#include "../Filter/Filter.hpp"
#include "../InputSource/InputSource.hpp"
#include "../SystemController/SystemController.hpp"
#include "../MotorController/MotorController.hpp"

class LineFollower
{
public:
  LineFollower() : LineFollower(nullptr, nullptr, nullptr) {}
  LineFollower(InputSource *pInputSource, SystemController *pSystemController, MotorController *pMotorController);
  ~LineFollower();

  void setInputSource(InputSource *pInputSource) { m_pInputSource = pInputSource; }
  void setSystemController(SystemController *pSystemController) { m_pSystemController = pSystemController; }
  void setMotorController(MotorController *pMotorController) { m_pMotorController = pMotorController; }

  void start();
  void stop();
  void update();

  bool getIsRunning() { return m_IsRunning; }
  unsigned long getStartTime() { return m_StartTime; }
  unsigned long getStopTime() { return m_StopTime; }

  void setLinearVelocity(float linearVelocity) { m_LinearVelocity = linearVelocity; }
  float getLinearVelocity() { return m_LinearVelocity; }
private:
  bool m_IsRunning;
  float m_LinearVelocity;
  InputSource *m_pInputSource;
  SystemController *m_pSystemController;
  MotorController *m_pMotorController;

  unsigned long m_StartTime;
  unsigned long m_StopTime;
};

#endif //LineFollower_hpp
