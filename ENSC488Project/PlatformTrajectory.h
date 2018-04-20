#pragma once
#include <math.h>
#include <iostream>
#include "Global.h"
#include "Point3D.h"

class PlatformTrajectory
{
public:
	bool HasPath;
	bool IsTurning;

	PlatformTrajectory();
	~PlatformTrajectory();

	void CalculatePath(Point3D start, Point3D via, Point3D end, float viaSpeed, float time);
	Point3D NextPosition(float elapsedTime);

private:
	const float maxAccel = 200;
	float trajTime = 0;
	float totalTime = 0;

	struct coefficients
	{
		float a0, a1, a2, a3;
	};

	enum movementType
	{
		LINE,
		ARC
	};

	struct function
	{
		float startTime;
		float timeSpan;
		movementType type;
		Point2D centerPoint;
		bool isPositive;
		coefficients coef;
	};

	function functions[18]; // handles maximum 9 functions
	int functionIter = 0;

	coefficients findCoefficients(float pos0, float posf, float speed0, float speedf, float time);
	void addTrajectory(Point3D start, Point3D end, float startSpeed, float endSpeed, float time);
	float evaluateFunction(function f, float t);
};