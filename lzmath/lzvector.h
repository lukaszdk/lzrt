#ifndef _LZMATH_LZVECTOR_H_
#define _LZMATH_LZVECTOR_H_

#include <iostream>
#include <lzpoint.h>
#include <lznormal.h>

class lzmath::Vector3f
{
	public:
			float x;
			float y;
			float z;
			float w;

	public:
		/* Constructors */
		Vector3f();
		Vector3f(float x , float y, float z);
		Vector3f(const lzmath::Point3f &p);
		Vector3f(const lzmath::Normal &n);
		/* Operators */
		Vector3f operator+(const Vector3f &v) const;
		Vector3f operator-(const Vector3f &v) const;
		Vector3f operator-() const;
		Vector3f operator*(const float scalar) const;
		Vector3f operator/(const float scalar) const;
		float operator[](unsigned int i) const;
		float Length() const;
};

namespace lzmath
{
	class Vector3f;
	class Normal;

	inline float Dot(const Vector3f &v1, const Vector3f &v2);
	Vector3f Cross(const Vector3f &v1, const Vector3f &v2);
	Vector3f Normalize(const Vector3f &v);
	inline void Cross(Vector3f *result, Vector3f *v1, Vector3f *v2);
	inline void Subtract(Vector3f *result, Vector3f *v1, Vector3f *v2);
	inline void Subtract(Vector3f *result, Point3f *p1, Point3f *p2);
}


inline void lzmath::Subtract(Vector3f *result, Point3f *p1, Point3f *p2)
{
	result->x = p1->x - p2->x;
	result->y = p1->y - p2->y;
	result->z = p1->z - p2->z;
}

// result = v1 - v2
inline void lzmath::Subtract(Vector3f *result, Vector3f *v1, Vector3f *v2)
{
	result->x = v1->x - v2->x;
	result->y = v1->y - v2->y;
	result->z = v1->z - v2->z;
}


inline void lzmath::Cross(Vector3f *result, Vector3f *v1, Vector3f *v2)
{
	result->x = (v1->y * v2->z) - (v1->z * v2->y);
	result->y = (v1->z * v2->x) - (v1->x * v2->z);
	result->z = (v1->x * v2->y) - (v1->y * v2->x);
}

inline float lzmath::Dot(const Vector3f &v1, const Vector3f &v2)
{
	return v1.x * v2.x + v1.y * v2.y + v1.z *v2.z;
}


inline std::ostream &operator<<(std::ostream &os, const lzmath::Vector3f &v)
{
	os << "Vector3f(" << v.x << "," << v.y << "," << v.z << ")";

	return os;
}


#endif

