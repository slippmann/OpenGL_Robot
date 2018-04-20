#include "Global.h"
float toDegrees(float radians)
{
	return (radians * (180.0 / PI));
}

float toRadians(float degrees)
{
	return (degrees * (PI / 180.0));
}

float Cos(float angle)
{
	return cos(toRadians(angle));
}

float aCos(float angle)
{
	return toDegrees(acos(angle));
}

float Sin(float angle)
{
	return sin(toRadians(angle));
}

float aSin(float angle)
{
	return toDegrees(asin(angle));
}

float Tan(float angle)
{
	return tan(toRadians(angle));
}

float aTan2(float y, float x)
{
	return toDegrees(atan2(y, x));
}