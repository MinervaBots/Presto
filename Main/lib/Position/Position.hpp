#ifndef Position_hpp
#define Position_hpp

class Position
{
public:
  Position() : Position(0, 0, 0) {}
  Position(float x, float y, float heading);
  float getX() { return m_X; }
  float getY() { return m_Y; }
  float getHeading() { return m_Heading; }

  void setX(float x) { m_X = x; }
  void setY(float y) { m_Y = y; }
  void setHeading(float heading) { m_Heading = heading; }

  static float distance(Position position1, Position position2);
  static Position gap(Position position1, Position position2);
private:
  float m_X;
  float m_Y;
  float m_Heading;
};

#endif // Position_hpp
