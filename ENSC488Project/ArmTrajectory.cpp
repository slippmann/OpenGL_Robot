#include "ArmTrajectory.h"

ArmTrajectory::ArmTrajectory()
{
	
};

ArmTrajectory::~ArmTrajectory()
{
	
};

bool ArmTrajectory::CalculateArmPath(Point3D startPos, Point3D viaPos, Point3D goalPos, float speed, float time)
{
	trajTime = time;

	float x = startPos.x - goalPos.x;
	float y = startPos.y - goalPos.y;
	float z = startPos.z - goalPos.z;
	
	float dist = sqrt(x*x + y*y + z*z);
	
	if (dist > 100)
	{
		//arm cant reach here from current position. 
		//call PathTrajectory();
		return false;
	}
	
	//calculate the coefficients for the 6 parameters 
	funcXvia.coef = findCoefficients(startPos.x, viaPos.x, 0, speed, time /2.0);
	funcYvia.coef = findCoefficients(startPos.y, viaPos.y, 0, speed, time /2.0);
	funcZvia.coef = findCoefficients(startPos.z, viaPos.z, 0, speed, time /2.0);
	/* funcT4via.coef = findCoefficients(startOrient.x, viaPos.x, 0, speed, time); 
	funcT5via.coef = findCoefficients(startOrient.y, viaPos.x, 0, speed, time);
	funcT6via.coef = findCoefficients(startOrient.z, viaPos.x, 0, speed, time); */
	
	funcXgoal.coef = findCoefficients(viaPos.x, goalPos.x, speed, 0, time /2.0);
	funcYgoal.coef = findCoefficients(viaPos.y, goalPos.y, speed, 0, time /2.0);
	funcZgoal.coef = findCoefficients(viaPos.z, goalPos.z, speed, 0, time /2.0);
	/* funcT4via.coef = findCoefficients(startOrient.x, viaPos.x, 0, speed, time); 
	funcT5via.coef = findCoefficients(startOrient.y, viaPos.x, 0, speed, time);
	funcT6via.coef = findCoefficients(startOrient.z, viaPos.x, 0, speed, time); */

	return true;
};

ArmTrajectory::coefficients ArmTrajectory::findCoefficients(float pos0, float posf, float speed0, float speedf, float time)
{
	coefficients coef;
	coef.timespan = time;
	coef.a0 = pos0;
	coef.a1 = speed0;
	coef.a2 = (3.0 / (time*time))*(posf - pos0) - (2.0 / time)*speed0 - (1.0 / time)*speedf;
	coef.a3 = -(2.0 / (time*time*time))*(posf - pos0) + (1.0 / (time*time))*(speedf + speed0);

	return coef;
};

Point3D ArmTrajectory::GiveArmPos(float time)
{
	if (time >= trajTime/2.0)
	{
		float t = time - trajTime / 2.0;
		float x = funcXgoal.coef.a0 + funcXgoal.coef.a1*t + funcXgoal.coef.a2*t*t + funcXgoal.coef.a3*t*t*t;
		float y = funcYgoal.coef.a0 + funcYgoal.coef.a1*t + funcYgoal.coef.a2*t*t + funcYgoal.coef.a3*t*t*t;
		float z = funcZgoal.coef.a0 + funcZgoal.coef.a1*t + funcZgoal.coef.a2*t*t + funcZgoal.coef.a3*t*t*t;
		
		Point3D nextPos(x, y, z);
		return nextPos;
	}
	
	else
	{
		float x = funcXvia.coef.a0 + funcXvia.coef.a1*time + funcXvia.coef.a2*time*time + funcXvia.coef.a3*time*time*time;
		float y = funcYvia.coef.a0 + funcYvia.coef.a1*time + funcYvia.coef.a2*time*time + funcYvia.coef.a3*time*time*time;
		float z = funcZvia.coef.a0 + funcZvia.coef.a1*time + funcZvia.coef.a2*time*time + funcZvia.coef.a3*time*time*time;
		
		Point3D nextPos(x, y, z);
		return nextPos;
	}
};