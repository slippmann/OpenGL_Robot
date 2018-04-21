#include "Inertia.h"

Inertia::Inertia()
{

};

Inertia::~Inertia()
{

}
bool Inertia::IsTipping(float startMass, Point3D endPos, float endMass)
{
	float distance = sqrt(pow((endPos.x),2) + pow((endPos.y),2));

	float length = (endMass * distance) / (endMass + startMass);
	
	if (length > 10)
	{
		return true;
	}

	else
	{
		return false;
	}
}