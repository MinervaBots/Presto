#ifndef WheelEncoder_h
#define WheelEncoder_h

class WheelEncoder
{
public:
  WheelEncoder(int leftTickPin, int rightTickPin, float wheelRadius, int ticksPerRevolution);
  void update();
  float getTotalDistance();
  float getTotalDistanceRight();
  float getTotalDistanceLeft();

  float getDeltaDistance();
  float getDeltaDistanceRight();
  float getDeltaDistanceLeft();

private:
  static WheelEncoder *_pInstance;

  //float _tickArc;
  float _wheelRadius;
  int _ticksPerRevolution;

  int _deltaLeftTickCount;
  float _deltaDistanceLeft;
  float _totalDistanceLeft;

  int _deltaRightTickCount;
  float _deltaDistanceRight;
  float _totalDistanceRight;

  static void leftTick();
  static void rightTick();
};

#endif // WheelEncoder_h
