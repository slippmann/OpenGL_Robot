#include "PlatformTrajectory.h"

PlatformTrajectory::PlatformTrajectory()
{
}

void PlatformTrajectory::CalculatePath(Point3D start, Point3D via, Point3D end, float viaSpeed, float time)
{
	float aLen = sqrt(pow((via.x - start.x), 2) + pow((via.y - start.y), 2));
	float bLen = sqrt(pow((end.x - via.x), 2) + pow((end.y - via.y), 2));

	trajTime = 0;
	IsTurning = false;

	functionIter = 0; // reset function iterator

	float aTime = time * (aLen / (aLen + bLen));
	float bTime = time - aTime;

	addTrajectory(start, via, 0, viaSpeed, aTime);
	addTrajectory(via, end, viaSpeed, 0, bTime);

#ifdef DEBUG
	for (int i = 0; i < functionIter; i++)
	{
		std::cout << "functions[" << i << "]:" << std::endl; 
		std::cout << "\t" << "startTime: " << functions[i].startTime << std::endl;
		std::cout << "\t" << "timeSpan:  " << functions[i].timeSpan << std::endl;
		std::cout << "\t" << "coef:      " << functions[i].coef.a0 << ", " << functions[i].coef.a1 << ", " << functions[i].coef.a2 << ", " << functions[i].coef.a3 << std::endl;
	}
#endif

	totalTime = 0;
	HasPath = true;
}

Point3D PlatformTrajectory::NextPosition(float elapsedTime)
{
	Point3D position;

	if (!HasPath)
	{
		std::cout << "ERROR: Trajectory is not set..." << std::endl;
		return Point3D();
	}

	totalTime += elapsedTime;

	int i = 0;
	float t;
	
	while ((totalTime > functions[i].startTime + functions[i].timeSpan) && 
		   (totalTime < trajTime) && 
		   (i < functionIter))
	{
		i++;
	}

	if (i == functionIter || totalTime >= trajTime)
	{
		HasPath = false;
		i = functionIter - 1;
		t = functions[i].timeSpan;
	}
	else
	{
		t = totalTime - functions[i].startTime;
	}

	if (IsTurning && i > 3)
	{
		IsTurning = false;
	}

	if (functions[i].type == LINE)
	{
		position.x = evaluateFunction(functions[i], t);
		position.y = evaluateFunction(functions[i + 1], t);
		position.z = NAN;
	}
	else
	{
		position.z = evaluateFunction(functions[i], t);
		position.x = functions[i].centerPoint.x;
		position.y = functions[i].centerPoint.y;

		if (!IsTurning)
		{
			if (functions[i].isPositive)
			{
				position.x += 10 * Sin(position.z);
				position.y -= 10 * Cos(position.z);
			}
			else
			{
				position.x -= 10 * Sin(position.z);
				position.y += 10 * Cos(position.z);
			}
		}
	}

#ifdef DEBUG
	std::cout << "Time: " << totalTime << std::endl;
	std::cout << "Location: " << position.x << ", " << position.y << ", " << position.z << std::endl;
#endif

	return position;
}

PlatformTrajectory::~PlatformTrajectory()
{
}

PlatformTrajectory::coefficients PlatformTrajectory::findCoefficients(float pos0, float posf, float speed0, float speedf, float time)
{
	coefficients coef;
	coef.a0 = pos0;
	coef.a1 = speed0;
	coef.a2 = (3.0 / (time*time))*(posf - pos0) - (2.0 / time)*speed0 - (1.0 / time)*speedf;
	coef.a3 = -(2.0 / (time*time*time))*(posf - pos0) + (1.0 / (time*time))*(speedf + speed0);

	return coef;
}

void PlatformTrajectory::addTrajectory(Point3D start, Point3D end, float startSpeed, float endSpeed, float time)
{
	function linexFunc;
	function lineyFunc;
	function arcFunc;

	if (time <= 0 || start == end)
	{
		return;
	}

	float len = sqrt(pow((end.x - start.x), 2) + pow((end.y - start.y), 2));

	// complex case
	if (len > 0)
	{
		if (end.z > 180)
		{
			end.z -= 360;
		}

		// circle center point
		float cx1 = end.x - (10 * Sin(end.z));
		float cy1 = end.y + (10 * Cos(end.z));

		float cx2 = end.x + (10 * Sin(end.z));
		float cy2 = end.y - (10 * Cos(end.z));

		// length from robot to c
		float a1 = sqrt(pow((cx1 - start.x), 2) + pow((cy1 - start.y), 2));
		float a2 = sqrt(pow((cx2 - start.x), 2) + pow((cy2 - start.y), 2));

		float a = (a1 < a2) ? a1 : a2;
		float cx = (a1 < a2) ? cx1 : cx2;
		float cy = (a1 < a2) ? cy1 : cy2;

		// length from robot to Tangent point
		float b = sqrt(pow(a, 2) - (10 * 10));

		// angles to determine absolute positions
		float thetaA = aTan2((cy - start.y), (cx - start.x));
		float thetaAB = aSin(10 / a);
		float thetaB = (end.z > thetaA) ? (thetaA - thetaAB) : (thetaA + thetaAB);

		arcFunc.centerPoint = Point2D(cx, cy);
		arcFunc.isPositive = (end.z > thetaA);

		// Tangent point to arc
		float tx = start.x + (b * Cos(thetaB));
		float ty = start.y + (b * Sin(thetaB));

		float angleDiff = abs(end.z - thetaB);
		float arcLen = 10 * toRadians(angleDiff);

		float totalLen = b + arcLen;

		coefficients velocitySpline = findCoefficients(0, totalLen, startSpeed, endSpeed, time);

		float tv = time * (b / totalLen);
		float viaVelocity = velocitySpline.a1 + velocitySpline.a2*tv + velocitySpline.a3*tv*tv;
			
		float startOmega =  viaVelocity / 10;
		float endOmega = endSpeed / 10;

		if (start.z != thetaB)
		{
			Point3D newStart = Point3D(start.x, start.y, thetaB);
			float turnTime = abs(thetaB - start.z) / 180; // extra one second for each 180 degrees

			addTrajectory(start, newStart, 0, 0, turnTime);

			IsTurning = true;

			start = newStart;
		}

		linexFunc.timeSpan = time * (b / totalLen);
		lineyFunc.timeSpan = linexFunc.timeSpan;
		arcFunc.timeSpan = time - linexFunc.timeSpan;

		linexFunc.type = LINE;
		lineyFunc.type = LINE;
		arcFunc.type = ARC;

		lineyFunc.startTime = trajTime;
		linexFunc.startTime = trajTime;
		arcFunc.startTime = trajTime + linexFunc.timeSpan;

		linexFunc.coef = findCoefficients(start.x, tx, (startSpeed * Cos(thetaB)), (viaVelocity * Cos(thetaB)), linexFunc.timeSpan);
		lineyFunc.coef = findCoefficients(start.y, ty, (startSpeed * Sin(thetaB)), (viaVelocity * Sin(thetaB)), lineyFunc.timeSpan);
		arcFunc.coef = findCoefficients(start.z, end.z, startOmega, endOmega, arcFunc.timeSpan);
	}
	// generic case
	else
	{
		float theta = aTan2((end.y - start.y), (end.x - start.x));

		linexFunc.startTime = trajTime;
		linexFunc.timeSpan = (len == 0) ? 0 : time;
		linexFunc.type = LINE;

		lineyFunc.startTime = trajTime;
		lineyFunc.timeSpan = linexFunc.timeSpan;
		lineyFunc.type = LINE;

		arcFunc.startTime = trajTime;
		arcFunc.timeSpan = (start.z == end.z) ? 0 : time;
		arcFunc.type = ARC;
		arcFunc.centerPoint = Point2D(start.x, start.y);

		linexFunc.coef = findCoefficients(start.x, end.x, (startSpeed * Cos(theta)), (endSpeed * Cos(theta)), time);
		lineyFunc.coef = findCoefficients(start.y, end.y, (startSpeed * Sin(theta)), (endSpeed * Sin(theta)), time);
		arcFunc.coef = findCoefficients(start.z, end.z, 0, 0, arcFunc.timeSpan);
	}

	functions[functionIter++] = linexFunc;
	functions[functionIter++] = lineyFunc;
	functions[functionIter++] = arcFunc;

	trajTime += time;
}

float PlatformTrajectory::evaluateFunction(function f, float t)
{
	return (f.coef.a0 + f.coef.a1 * t + f.coef.a2*t*t + f.coef.a3*t*t*t);
}
