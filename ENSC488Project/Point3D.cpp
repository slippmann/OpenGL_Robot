#include "Point3D.h"

Point3D::Point3D()
{
	x = 0;
	y = 0;
	z = 0;
}

Point3D::Point3D(float px, float py, float pz) : Point2D(px, py)
{
	z = pz;
}

Point3D::Point3D(Point3D & pt)
{
	x = pt.x;
	y = pt.y;
	z = pt.z;
}

Point3D::Point3D(Point2D & pt) : Point2D(pt)
{
	z = 0;
}

Point3D::Point3D(Point3D & pt, float offsetx, float offsety, float offsetz)
{
	x = pt.x + offsetx;
	y = pt.y + offsety;
	z = pt.z + offsetz;
}

Point3D::Point3D(Point2D & pt, float offsetx, float offsety, float offsetz) : Point2D(pt, offsetx, offsety)
{
	z = offsetz;
}


Point3D::~Point3D()
{
}
