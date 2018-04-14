#include "Colour.h"



Colour::Colour()
{
	this->r = 0.0;
	this->g = 0.0;
	this->b = 0.0;
}

Colour::Colour(unsigned char r, unsigned char g, unsigned char b)
{
	this->r = r;
	this->g = g;
	this->b = b;
}


Colour::~Colour()
{
}
