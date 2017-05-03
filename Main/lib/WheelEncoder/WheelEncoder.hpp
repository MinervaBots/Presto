#ifndef WheelEncoder_hpp
#define WheelEncoder_hpp

class WheelEncoder
{
public:
  WheelEncoder() {}
  WheelEncoder(int leftTickPin, int rightTickPin, float wheelRadius, unsigned int ticksPerRevolution);
  void update();
  void setWheelRadius(float wheelRadius) { m_WheelRadius = wheelRadius; }
  void setTicksPerRevolution(unsigned int ticksPerRevolution) { m_TicksPerRevolution = ticksPerRevolution; }

  float getTotalDistance() { return ((getTotalDistanceRight() + getTotalDistanceLeft()) / 2); }
  float getTotalDistanceRight() { return m_TotalDistanceLeft; }
  float getTotalDistanceLeft() { return m_TotalDistanceRight; }

  float getDeltaDistance() { return ((getDeltaDistanceRight() + getDeltaDistanceLeft()) / 2); }
  float getDeltaDistanceRight() { return m_DeltaDistanceLeft; }
  float getDeltaDistanceLeft() { return m_DeltaDistanceRight; }

private:
  static WheelEncoder *m_pInstance;

  float m_WheelRadius;
  unsigned int m_TicksPerRevolution;

  volatile int m_DeltaLeftTickCount;
  float m_DeltaDistanceLeft;
  float m_TotalDistanceLeft;

  volatile int m_DeltaRightTickCount;
  float m_DeltaDistanceRight;
  float m_TotalDistanceRight;

  static void leftTick();
  static void rightTick();
};

#endif // WheelEncoder_hpp
