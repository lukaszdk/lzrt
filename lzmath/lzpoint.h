#ifndef _LZMATH_LZPOINT_H_
#define _LZMATH_LZPOINT_H_

#include <iostream>

namespace lzmath
{
	class Point3f;
	class Vector3f;
}

class lzmath::Point3f
{
	public:
		float x,y,z;
	public:
		/* Constructors */
		Point3f(float x = 0, float y = 0, float z = 0);
		Point3f(const lzmath::Vector3f &v);
		/* Operators */
		Point3f operator+(const Point3f &p) const;
		Point3f operator+(const lzmath::Vector3f &v) const;
		Point3f operator-(const Point3f &p) const;
		Point3f operator-(const lzmath::Vector3f &v) const;
		Point3f operator-() const;
		Point3f operator*(float scalar) const;
		Point3f operator/(float scalar) const;
		float operator[](unsigned int i) const;
		float& operator[](unsigned int i);
		void Clone(Point3f p);
};

inline std::ostream &operator<<(std::ostream &os, const lzmath::Point3f &p)
{
	os << "Point3f(" << p.x << "," << p.y << "," << p.z << ")";

	return os;
}


#endif

