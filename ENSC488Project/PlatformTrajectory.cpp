#include "PlatformTrajectory.h"

PlatformTrajectory::PlatformTrajectory()
{
}

void PlatformTrajectory::CalculatePath(Point3D start, Point3D via, Point3D end, float startAngle, float endAngle, float viaSpeed, float time)
{
	float angle = atan2((via.y - start.y) , (via.x - start.x));

	float aLen = sqrt(pow((via.x - start.x), 2) + pow((via.y - start.y), 2));
	float bLen = sqrt(pow((end.x - via.x), 2) + pow((end.y - via.y), 2));

	if (startAngle != angle)
	{
		turnTime = 0.1 * time; // ten percent of total time spent 
		time *= 0.9;

		addTrajectory(start, start, 0, 0, turnTime);
	}

	float aTime = time * (aLen / (aLen + bLen));
	float bTime = time - aTime;

	addTrajectory(start, via, 0, viaSpeed, aTime);
	addTrajectory(via, end, viaSpeed, 0, bTime);

#ifdef DEBUG
	std::cout << "xPosv: " << xPosv.a0 << ", " << xPosv.a1 << ", " << xPosv.a2 << ", " << xPosv.a3 << std::endl;
	std::cout << "xPosf: " << xPosf.a0 << ", " << xPosf.a1 << ", " << xPosf.a2 << ", " << xPosf.a3 << std::endl;

	std::cout << "yPosv: " << yPosv.a0 << ", " << yPosv.a1 << ", " << yPosv.a2 << ", " << yPosv.a3 << std::endl;
	std::cout << "yPosf: " << yPosf.a0 << ", " << yPosf.a1 << ", " << yPosf.a2 << ", " << yPosf.a3 << std::endl;

	std::cout << "zAnglev: " << zAnglev.a0 << ", " << zAnglev.a1 << ", " << zAnglev.a2 << ", " << zAnglev.a3 << std::endl;
	std::cout << "zAnglef: " << zAnglef.a0 << ", " << zAnglef.a1 << ", " << zAnglef.a2 << ", " << zAnglef.a3 << std::endl;
#endif

	totalTime = 0;
	IsSet = true;
}

Point3D PlatformTrajectory::NextPosition(float elapsedTime)
{
#if 0
	Point3D position;

	totalTime += elapsedTime;

	if (totalTime <= turnTime)
	{
		position.z = zAnglev.a0 + zAnglev.a1*t + zAnglev.a2*t*t + zAnglev.a3*t*t*t;
	}
	else if (totalTime <= (turnTime + aTime))
	{
		Location.x = xPosv.a0 + xPosv.a1*t + xPosv.a2*t*t + xPosv.a3*t*t*t;
		Location.y = yPosv.a0 + yPosv.a1*t + yPosv.a2*t*t + yPosv.a3*t*t*t;
	}
	else
	{
		t2 = t - trajTime / 2;

		Location.x = xPosf.a0 + xPosf.a1*t2 + xPosf.a2*t2*t2 + xPosf.a3*t2*t2*t2;
		Location.y = yPosf.a0 + yPosf.a1*t2 + yPosf.a2*t2*t2 + yPosf.a3*t2*t2*t2;
	}

#ifdef DEBUG
	std::cout << "Time: " << t << std::endl;
	std::cout << "Location: " << Location.x << ", " << Location.y << std::endl;
#endif

	if (totalTime >= trajTime)
	{
		IsSet = false;
		return = end;
	}
#endif

	return Point3D();
}

PlatformTrajectory::~PlatformTrajectory()
{
}

PlatformTrajectory::coefficients PlatformTrajectory::findCoefficients(float pos0, float posf, float speed0, float speedf, float time)
{
	coefficients coef;
	coef.timespan = time;
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

	// circle center point
	float cx = end.x - (10 * cos(end.z));
	float cy = end.y + (10 * sin(end.z));

	// length from robot to c
	float a = sqrt(pow((cx - start.x), 2) + pow((cy - start.y), 2));

	// length from robot to tangent point
	float b = sqrt(pow(a, 2) - 10 * 10);

	// angles to determine absolute positions
	float thetaA = acos((cx - start.x) / a);
	float thetaAB = asin(10 / a);
	float thetaB = thetaAB - thetaA;

	// tangent point to arc
	float tx = start.x + (b * cos(thetaB));
	float ty = start.y + (b * sin(thetaB));

	float startOmega = startSpeed * 10;
	float endOmega = endSpeed * 10;

	float angleDiff = end.z + thetaB;
	float arcLen = 20 * PI * (angleDiff / 360);

	float totalLen = b + arcLen;

	linexFunc.timeSpan = time * (b / totalLen);
	lineyFunc.timeSpan = linexFunc.timeSpan;

	arcFunc.timeSpan = time - travelTime;

	linexFunc.coef = findCoefficients(start.x, end.x, (startSpeed * cos(thetaB)), (startSpeed * cos(thetaB)), linexFunc.timeSpan);
	lineyFunc.coef = findCoefficients(start.y, end.y, (startSpeed * sin(thetaB)), (startSpeed * sin(thetaB)), lineyFunc.timeSpan);
	arcFunc.coef = findCoefficients(start.z, end.z, startOmega, endOmega, arcFunc.timeSpan);

	functions[functionIter++] = linexFunc;
	functions[functionIter++] = lineyFunc;
	functions[functionIter++] = arcFunc;
}
