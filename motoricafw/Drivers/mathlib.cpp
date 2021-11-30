#include "stdint.h" 
#include "mathlib.h"

float linearInterp(float* x, float* y, uint8_t size, float inputValue)
{
	uint8_t i = 0;
	float v1, v2, p1, p2, p0, a, b;
	while (inputValue > x[i])
	{
		i++;
		if (i == size)
			break;
	}
	if (i < 1)
	{
		return 0;
	}
	else if (i == size)
	{
		v1 = x[i - 1];
		v2 = x[i - 1] - x[i - 2] + x[i - 1];
		p1 = y[i - 1];
		p2 = y[i - 1] - y[i - 2] + y[i - 1];
	}
	else
	{
		v1 = x[i - 1];
		v2 = x[i];
		p1 = y[i - 1];
		p2 = y[i];
	}
	a = (p2 - p1) / (v2 - v1);
	b = p1 - a * v1;
	p0 = a * inputValue + b;
	return p0;
}