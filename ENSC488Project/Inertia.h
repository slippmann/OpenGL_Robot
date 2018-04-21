#pragma once
#include <math.h>
#include <iostream>
#include "Global.h"
#include "Point3D.h"

class Inertia
{
public:
	Inertia();
	~Inertia();
	bool IsTipping(float startMass, Point3D endPos, float endMass);
};