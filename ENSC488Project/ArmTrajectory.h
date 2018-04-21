#pragma once
#include <math.h>
#include "Point3D.h"
#define PI				3.14159265358979323846

class ArmTrajectory
{
public:
	float trajTime = 1;

	ArmTrajectory();
	~ArmTrajectory();
	bool CalculateArmPath(Point3D startPos, Point3D viaPos, Point3D goalPos, float speed, float time);
	Point3D GiveArmPos(float time);

private:
	float speed;
	bool viaDone = false;

	struct coefficients
	{
		float timespan;
		float a0, a1, a2, a3;
	};

	struct function
	{
		float timeSpan;
		struct coefficients coef;
	};
	
	function funcXvia, funcYvia, funcZvia, funcT4via, funcT5via, funcT6via;
	function funcXgoal, funcYgoal, funcZgoal, funcT4goal, funcT5goal, funcT6goal;
	
	coefficients findCoefficients(float pos0, float posf, float speed0, float speedf, float time);
	
	float evaluateFuntion(function f, float t)
	{
		return (f.coef.a0 + f.coef.a1*t + f.coef.a2*t*t + f.coef.a3*t*t*t);
	}
};