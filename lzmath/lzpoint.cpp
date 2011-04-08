#include <lzpoint.h>
#include <lzvector.h>

using namespace lzmath;

Point3f::Point3f(float x, float y, float z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

Point3f::Point3f(const lzmath::Vector3f &v)
{
	x = v.x;
	y = v.y;
	z = v.z;
}

Point3f Point3f::operator+(const Point3f &p) const
{
	return Point3f(x + p.x, y + p.y, z + p.z);
}

Point3f Point3f::operator+(const Vector3f &v) const
{
	return Point3f(x + v.x, y + v.y, z + v.z);
}

Point3f Point3f::operator-(const Point3f &p) const
{
	return Point3f(x - p.x, y - p.y, z - p.z);
}

Point3f Point3f::operator-() const
{
	return Point3f(-x, -y, -z);
}

Point3f Point3f::operator-(const lzmath::Vector3f &v) const
{
	return Point3f(x - v.x, y - v.y, z - v.z);
}


Point3f Point3f::operator*(float scalar) const
{
	return Point3f(x * scalar, y * scalar, z * scalar);
}

Point3f Point3f::operator/(float scalar) const
{
	float invscalar = 1 / scalar;

	return Point3f(x * invscalar, y * invscalar, z * invscalar);
}

float Point3f::operator[](unsigned int i) const
{
	switch(i)
	{
		case 0: return x;
		case 1: return y;
		case 2: return z;
		default: return x;
	}
}

float& Point3f::operator[](unsigned int i)
{
	switch(i)
	{
		case 0: return x;
		case 1: return y;
		case 2: return z;
		default: return x;
	}
}

void Point3f::Clone(Point3f p)
{
	x = p.x;
	y = p.y;
	z = p.z;
}


