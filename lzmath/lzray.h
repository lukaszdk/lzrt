#ifndef _LZMATH_LZRAY_H_
#define _LZMATH_LZRAY_H_

#define LZRAY_EPSILON		1e-3f
#define LZRAY_INFINITY 	3.40282347e+38

#include <lzmath.h>

namespace lzmath
{
	class Ray;	
}

class lzmath::Ray
{
	public:
		lzmath::Point3f o; // origin
		lzmath::Vector3f d; // direction;
		mutable float mint, maxt;
	public:	
		Ray();
		Ray(const lzmath::Point3f &origin, const lzmath::Vector3f &direction, 
				float start = LZRAY_EPSILON, float end = LZRAY_INFINITY);
		void Clone(Ray &ray);
		void Reset();

		lzmath::Point3f operator()(float t) const;
};

inline std::ostream &operator<<(std::ostream &os, const lzmath::Ray &r)
{
	os << "Ray o(" << r.o.x << "," << r.o.y << "," << r.o.z << "), d(" << r.d.x << "," << r.d.y << "," << r.d.z << ")";

	return os;
}




#endif


