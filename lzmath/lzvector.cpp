#include <lzvector.h>
#include <lzpoint.h>
#include <lznormal.h>
#include <math.h>

#ifdef __SSE3__
#include <pmmintrin.h>
#endif 
using namespace lzmath;

Vector3f::Vector3f()
{

}

Vector3f::Vector3f(float x, float y, float z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

Vector3f::Vector3f(const lzmath::Point3f &p)
{
	x = p.x;
	y = p.y;
	z = p.z;
}

Vector3f::Vector3f(const lzmath::Normal &n)
{
	x = n.x;
	y = n.y;
	z = n.z;
}


Vector3f Vector3f::operator+(const Vector3f &v) const
{
	return Vector3f(x + v.x, y + v.y, z + v.z);
}

Vector3f Vector3f::operator-(const Vector3f &v) const
{
	return Vector3f(x - v.x, y - v.y, z - v.z);
}

Vector3f Vector3f::operator-() const
{
	return Vector3f(-x, -y, -z);
}

Vector3f Vector3f::operator*(const float scalar) const
{
	return Vector3f(x*scalar, y*scalar, z*scalar);
}


Vector3f Vector3f::operator/(const float scalar) const
{
	float d = 1/scalar;

	return Vector3f(x*d, y*d, z*d);
}

float Vector3f::operator[](unsigned int i) const
{
	switch(i)
	{
		case 0: return x;
		case 1: return y;
		case 2: return z;
		default: return x;
	}
}


float Vector3f::Length() const
{
	return sqrtf(x*x + y*y + z*z);
}

Vector3f lzmath::Normalize(const Vector3f &v)
{
	return (v /  v.Length());
}



#ifdef __SSE3__

void crossp_sse(const float v1[4], const float v2[4], float out[4]) 
{
	__m128 vector1, vector2, vector3, vector4, vector5;

	vector1 = _mm_load_ps(v1);
	vector2 = _mm_load_ps(v2);

	vector3 = _mm_shuffle_ps(vector2, vector1, _MM_SHUFFLE(3, 0, 2, 2));
	vector4 = _mm_shuffle_ps(vector1, vector2, _MM_SHUFFLE(3, 1, 0, 1));

	vector5 = _mm_mul_ps(vector3, vector4);

	vector3 = _mm_shuffle_ps(vector1, vector2, _MM_SHUFFLE(3, 0, 2, 2));
	vector4 = _mm_shuffle_ps(vector2, vector1, _MM_SHUFFLE(3, 1, 0, 1));

	vector3 = _mm_mul_ps(vector3, vector4);
	vector3 = _mm_sub_ps(vector5, vector3);

	_mm_store_ps(out, vector3);

	out[1] *= -1;
}
#endif

Vector3f lzmath::Cross(const Vector3f &v1, const Vector3f &v2)
{
	/*#ifdef __SSE3__
		Vector3f ret;

		crossp_sse( v1.vec, v2.vec, ret.vec);

		return ret;
	#else */

	return Vector3f(	(v1.y * v2.z) - (v1.z * v2.y),
						(v1.z * v2.x) - (v1.x * v2.z),
						(v1.x * v2.y) - (v1.y * v2.x));
	// #endif
}






