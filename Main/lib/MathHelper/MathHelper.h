#ifndef MathHelper_h
#define MathHelper_h

inline float clamp(float value, float min, float max)
{
	if (value < min)
		return min;
	else if (value > max)
		return max;
	return value;
}

inline float lerp(float start, float end, float t)
{
	return ((1 - t) * start) + (end * t);
}

#endif // MathHelper_h
