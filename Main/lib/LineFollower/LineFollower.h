#ifndef LineFollower_h
#define LineFollower_h

#include "../InputSource/InputSource.h"
#include "../SystemController/SystemController.h"
#include "../MotorController/MotorController.h"

class LineFollower
{
public:
  LineFollower();
  LineFollower(InputSource *pInputSource, SystemController *pSystemController, MotorController *pMotorController);
  ~LineFollower();

  void setInputSource(InputSource *pInputSource) { _pInputSource = pInputSource; }
  void setSystemController(SystemController *pSystemController) { _pSystemController = pSystemController; }
  void setMotorController(MotorController *pMotorController) { _pMotorController = pMotorController; }

  void start();
  void stop();
  void update();

private:
  InputSource *_pInputSource;
  SystemController *_pSystemController;
  MotorController *_pMotorController;

  unsigned long _startTime;
  unsigned long _stopTime;
};

#endif //LineFollower_h
