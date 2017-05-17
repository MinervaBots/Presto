#ifndef MathHelper_h
#define MathHelper_h

#if defined(ARDUINO)

// Incluí aqui porque eu sei que o Arduino tem essa biblioteca.
// PIC e ARM provavelmente também tem, então é só adicionar a inclusão
#include <math.h>
#include <Arduino.h>

#else

inline float constrain(float value, float min, float max)
{
	if (value < min)
		return min;
	else if (value > max)
		return max;
	return value;
}

#endif

inline float lerp(float start, float end, float t)
{
	return ((1 - t) * start) + (end * t);
}
/*
inline float min(float a, float b)
{
	if(a < b) return a;
	return b;
}

inline float max(float a, float b)
{
	if(a > b) return a;
	return b;
}

inline float pow(float value, float power)
{
	if(power == 0)
		return 1;
	else if(power == 1)
		return value;

	for (int i = 0; i < power; i++)
	{
		value *= value;
	}
	return value;
}

inline float sqrt(int value)
{

}
*/
#endif // MathHelper_h
