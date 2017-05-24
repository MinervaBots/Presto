#ifndef MathHelper_h
#define MathHelper_h

#if defined(ARDUINO)

/*
Incluí aqui porque eu sei que o Arduino tem essa biblioteca.
PIC e ARM provavelmente também tem, então é só adicionar a inclusão
*/
#include <math.h>
#include <Arduino.h>

#else

/*
O Arduino já tem esse método, mas se algum dia movermos para outro microcontrolador
como PIC ou ARM, é bom manter

Clamp -> https://en.wikipedia.org/wiki/Clamping_(graphics)
*/
inline float constrain(float value, float min, float max)
{
	if (value < min)
		return min;
	else if (value > max)
		return max;
	return value;
}

#endif

/*
Interpolação linear entre 'start' e 'end' em função de 't'
Quanto 't == 0', retorna 'start'. 't == 1' retorna 'end'...
https://pt.wikipedia.org/wiki/Interpola%C3%A7%C3%A3o_linear
*/
inline float lerp(float start, float end, float t)
{
	return ((1 - t) * start) + (end * t);
}

#endif // MathHelper_h
