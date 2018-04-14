#pragma once
class Point2D
{
public:
	float x;
	float y;

	Point2D();
	Point2D(float px, float py);
	Point2D(Point2D &pt);
	Point2D(Point2D &pt, float offsetx, float offsety);
	~Point2D();

	bool operator==(const Point2D& pt)
	{
		return (this->x == pt.x && this->y == pt.y);
	}

	bool operator!=(const Point2D& pt)
	{
		return !(*this == pt);
	}

	void operator+(const Point2D& pt)
	{
		this->x += pt.x;
		this->y += pt.y;
	}

	void operator-(const Point2D& pt)
	{
		this->x -= pt.x;
		this->y -= pt.y;
	}
};

