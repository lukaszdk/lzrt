#ifndef _LZMATH_LZNORMAL_H_
#define _LZMATH_LZNORMAL_H_

namespace lzmath
{
	class Normal;
	class Vector3f;

	Normal Normalize(const Normal &n);
	float Dot(const Normal &n1, const Normal &n2);
}

class lzmath::Normal
{
	public:
		float x,y,z;
	public:
		Normal(float x = 0, float y = 0, float z = 0);
		Normal(const lzmath::Vector3f &v);
		/* Operators */
		Normal operator+(const Normal &n) const;
		Normal operator-(const Normal &n) const;
		Normal operator-() const;
		Normal operator*(const float scalar) const;
		Normal operator/(const float scalar) const;
		float Length() const;
};



#endif


