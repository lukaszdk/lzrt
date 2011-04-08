#include <lznormal.h>
#include <lzvector.h>
#include <math.h>

using namespace lzmath;

Normal::Normal(float x, float y, float z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

Normal::Normal(const lzmath::Vector3f &v)
{
	x = v.x;
	y = v.y;
	z = v.z;
}

Normal Normal::operator+(const Normal &n) const
{
	return Normal(x + n.x, y + n.y, z + n.z);	
}

Normal Normal::operator-(const Normal &n) const
{
	return Normal(x - n.x, y - n.y, z - n.z);	
}

Normal Normal::operator-() const
{
	return Normal(-x, -y, -z);	
}

Normal Normal::operator*(const float scalar) const
{
	return Normal(x*scalar, y*scalar, z*scalar);
}

Normal Normal::operator/(const float scalar) const
{
	float d = 1/scalar;

	return Normal(x*d, y*d, z*d);
}

float Normal::Length() const
{
	return sqrtf(x*x + y*y + z*z);
}

Normal lzmath::Normalize(const Normal &n)
{
	return (n /  n.Length());
}

float lzmath::Dot(const Normal &n1, const Normal &n2)
{
	return n1.x * n2.x + n1.y * n2.y + n1.z *n2.z;
}


