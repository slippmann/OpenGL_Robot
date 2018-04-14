#pragma once
#include "Point2D.h"
class Point3D : public Point2D
{
public:
	float z;

	Point3D();
	Point3D(float px, float py, float pz);
	Point3D(Point3D &pt);
	Point3D(Point2D &pt);
	Point3D(Point3D &pt, float offsetx, float offsety, float offsetz);
	Point3D(Point2D &pt, float offsetx, float offsety, float offsetz);
	~Point3D();

	bool operator==(const Point3D& pt)
	{
		return (this->x == pt.x && this->y == pt.y && this->z == pt.z);
	}

	bool operator!=(const Point3D& pt)
	{
		return !(*this == pt);
	}

	void operator+(const Point3D& pt)
	{
		this->x += pt.x;
		this->y += pt.y;
		this->z += pt.z;
	}

	void operator-(const Point3D& pt)
	{
		this->x -= pt.x;
		this->y -= pt.y;
		this->z -= pt.z;
	}
};

