#include "Trajectory.h"

Trajectory::Trajectory()
{
}

void Trajectory::CalculateTrajectory(Point3D start, Point3D via, Point3D end, float viaSpeed, float time)
{
	float aLen = sqrt();
	float bLen = sqrt();

	float aTime = time * (aLen / (aLen + bLen));
	float bTime = time - aTime;

	calculateSingleTrajectory(via, 0, viaSpeed, aTime);
	calculateSingleTrajectory(end, viaSpeed, 0, bTime);

#ifdef DEBUG
	std::cout << "xPosv: " << xPosv.a0 << ", " << xPosv.a1 << ", " << xPosv.a2 << ", " << xPosv.a3 << std::endl;
	std::cout << "xPosf: " << xPosf.a0 << ", " << xPosf.a1 << ", " << xPosf.a2 << ", " << xPosf.a3 << std::endl;

	std::cout << "yPosv: " << yPosv.a0 << ", " << yPosv.a1 << ", " << yPosv.a2 << ", " << yPosv.a3 << std::endl;
	std::cout << "yPosf: " << yPosf.a0 << ", " << yPosf.a1 << ", " << yPosf.a2 << ", " << yPosf.a3 << std::endl;

	std::cout << "zAnglev: " << zAnglev.a0 << ", " << zAnglev.a1 << ", " << zAnglev.a2 << ", " << zAnglev.a3 << std::endl;
	std::cout << "zAnglef: " << zAnglef.a0 << ", " << zAnglef.a1 << ", " << zAnglef.a2 << ", " << zAnglef.a3 << std::endl;
#endif

	elapsedTime = 0;
	isSet = true;
}

Point3D Trajectory::NextPosition(float time)
{
	if (t <= turnTime)
	{
		dir = zAnglev.a0 + zAnglev.a1*t + zAnglev.a2*t*t + zAnglev.a3*t*t*t;
	}
	else if (t <= (turnTime + travelTime))
	{
		Location.x = xPosv.a0 + xPosv.a1*t + xPosv.a2*t*t + xPosv.a3*t*t*t;
		Location.y = yPosv.a0 + yPosv.a1*t + yPosv.a2*t*t + yPosv.a3*t*t*t;
	}
	else if (t <= (turnTime + travelTime * 2))
	{
		t2 = t - trajTime / 2;

		Location.x = xPosf.a0 + xPosf.a1*t2 + xPosf.a2*t2*t2 + xPosf.a3*t2*t2*t2;
		Location.y = yPosf.a0 + yPosf.a1*t2 + yPosf.a2*t2*t2 + yPosf.a3*t2*t2*t2;
	}
	else
	{
		float oldDir = dir;

		dir = zAnglef.a0 + zAnglef.a1*t2 + zAnglef.a2*t2*t2 + zAnglef.a3*t2*t2*t2;

		leftTurn = (dir > oldDir);
	}

#ifdef DEBUG
	std::cout << "Time: " << t << std::endl;
	std::cout << "Location: " << Location.x << ", " << Location.y << std::endl;
#endif

	if (t >= trajTime)
	{
		isTrajSet = false;
		Location = end;
	}
}

Trajectory::~Trajectory()
{
}

Trajectory::coefficients Trajectory::findCoefficients(float pos0, float posf, float speed0, float speedf, float time)
{
	coefficients coef;
	coef.timespan = time;
	coef.a0 = pos0;
	coef.a1 = speed0;
	coef.a2 = (3.0 / (time*time))*(posf - pos0) - (2.0 / time)*speed0 - (1.0 / time)*speedf;
	coef.a3 = -(2.0 / (time*time*time))*(posf - pos0) + (1.0 / (time*time))*(speedf + speed0);

	return coef;
}

void Trajectory::calculateSingleTrajectory(Point3D start, Point3D end, float startSpeed, float endSpeed, float time)
{
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

	float omega = speed * 10;
	float angleDiff = end.z + thetaB;
	float arcLen = 20 * PI * (angleDiff / 360);

	float totalLen = b + arcLen;

	travelTime = time * (b / totalLen);
	turnTime = time - travelTime;

	//for x
	xPosf = findCoefficients(start.x, end.x, speed, 0, travelTime / 2);

	//for y
	yPosf = findCoefficients(start.y, end.y, speed, 0, travelTime / 2);

	//for angle about z
	zAnglef = findCoefficients(start.z, end.z, omega, 0, turnTime / 2);
}
