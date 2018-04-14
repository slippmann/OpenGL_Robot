#include "Point2D.h"

Point2D::Point2D()
{
	x = 0;
	y = 0;
}

Point2D::Point2D(float px, float py)
{
	x = px;
	y = py;
}

Point2D::Point2D(Point2D & pt)
{
	x = pt.x;
	y = pt.y;
}

Point2D::Point2D(Point2D & pt, float offsetx, float offsety)
{
	x = pt.x + offsetx;
	y = pt.y + offsety;
}


Point2D::~Point2D()
{
}
