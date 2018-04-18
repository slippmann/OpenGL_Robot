#pragma once
#include <math.h>
#include "Point3D.h"

#define PI				3.14159265358979323846

class Trajectory
{
public:
	float Time;
	bool IsSet;

	Trajectory();
	~Trajectory();

	void CalculateTrajectory(Point3D start, Point3D via, Point3D end, float viaSpeed, float time);
	Point3D NextPosition(float time);

private:
	float speed;
	float trajTime = 1;
	float elapsedTime = 0;
	float turnTime = 0;
	float travelTime = 0;

	struct coefficients
	{
		float timespan;
		float a0, a1, a2, a3;
	};

	coefficients functions[4];

	coefficients findCoefficients(float pos0, float posf, float speed0, float speedf, float time);
	void calculateSingleTrajectory(Point3D start, Point3D end, float startSpeed, float endSpeed, float time);
};