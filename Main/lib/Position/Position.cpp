#include "Position.hpp"
#include <math.h>

Position::Position(float x, float y, float heading)
{
  setX(x);
  setY(y);
  setHeading(heading);
}

float Position::distance(Position position1, Position position2)
{
  return sqrt(pow((position1.getX() - position2.getX()), 2) + pow(position1.getY() - position2.getY(), 2));
}

Position Position::gap(Position position1, Position position2)
{
  float gapX = position2.getX() - position1.getX();
	float gapY = position2.getY() - position1.getY();
	float gapHeading = position2.getHeading() - position1.getHeading();
	return Position(gapX, gapY, gapHeading);
}
