#pragma once
#include <math.h>
#include "Point3D.h"

#define PI				3.14159265358979323846

class PlatformTrajectory
{
public:
	bool IsSet;

	PlatformTrajectory();
	~PlatformTrajectory();

	void CalculatePath(Point3D start, Point3D via, Point3D end, float startAngle, float endAngle, float viaSpeed, float time);
	Point3D NextPosition(float elapsedTime);

private:
	float speed;
	float trajTime = 1;
	float totalTime = 0;
	float turnTime = 0;
	float travelTime = 0;

	struct coefficients
	{
		float timespan;
		float a0, a1, a2, a3;
	};

	struct function
	{
		float timeSpan;
		coefficients coef;
	};

	function functions[7]; // handles maximum 7 functions
	int functionIter = 0;

	coefficients findCoefficients(float pos0, float posf, float speed0, float speedf, float time);
	void addTrajectory(Point3D start, Point3D end, float startSpeed, float endSpeed, float time);
};