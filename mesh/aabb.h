#ifndef _AABB_
#define _AABB_

#include <lzmath.h>
#include <mesh/mesh.h>

using namespace lzmath;

class AABB
{
	public:
		aabb_t *aabb;
	public:
		AABB(aabb_t *aabb);
		void Set(aabb_t *aabb);
		void Reset();
		void Union(float p[3]);
		void Union(aabb_t *aabb);
		bool Intersect(frustum_t *f);
		bool Intersect(ray_t *ray, float *tmin, float *tmax);
		void Intersect(float *o, float *d,  float *tmin, float *tmax, int axis);
		bool IntersectPlane(float plane, int axis);
		bool ToLeft(float plane, int axis);
		bool ToRight(float plane, int axis);
		float Area();
		void AreaLeftRight(float plane, int axis, float *aleft, float *aright);
		void Print();
};

#endif

