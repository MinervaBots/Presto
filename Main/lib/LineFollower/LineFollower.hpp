#ifndef LineFollower_hpp
#define LineFollower_hpp

#include "../InputSource/InputSource.hpp"
#include "../SystemController/SystemController.hpp"
#include "../MotorController/MotorController.hpp"

class LineFollower
{
public:
  LineFollower();
  LineFollower(InputSource *pInputSource, SystemController *pSystemController, MotorController *pMotorController);
  ~LineFollower();

  void setInputSource(InputSource *pInputSource) { m_pInputSource = pInputSource; }
  void setSystemController(SystemController *pSystemController) { m_pSystemController = pSystemController; }
  void setMotorController(MotorController *pMotorController) { m_pMotorController = pMotorController; }

  void start();
  void stop();
  void update();

private:
  InputSource *m_pInputSource;
  SystemController *m_pSystemController;
  MotorController *m_pMotorController;

  unsigned long m_StartTime;
  unsigned long m_StopTime;
};

#endif //LineFollower_hpp
