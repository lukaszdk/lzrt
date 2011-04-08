#include <algorithm>
#include <cfloat>
#include <math.h>
#include <mesh/aabb.h>

using namespace std;

extern float min4(float v1, float v2, float v3, float v4);
extern float max4(float v1, float v2, float v3, float v4);


AABB::AABB(aabb_t *aabb)
{
	this->aabb = aabb;
}

void AABB::Set(aabb_t *aabb)
{
	this->aabb = aabb;
}

void AABB::Reset()
{
	aabb->min[0] = 1000000;
	aabb->min[1] = 1000000;
	aabb->min[2] = 1000000;

	aabb->max[0] = -1000000;
	aabb->max[1] = -1000000;
	aabb->max[2] = -1000000;

}

void AABB::Union(float p[3])
{
	aabb->min[0] = min(aabb->min[0], p[0]);
	aabb->min[1] = min(aabb->min[1], p[1]);
	aabb->min[2] = min(aabb->min[2], p[2]);

	aabb->max[0] = max(aabb->max[0], p[0]);
	aabb->max[1] = max(aabb->max[1], p[1]);
	aabb->max[2] = max(aabb->max[2], p[2]);
}

void AABB::Union(aabb_t *aabb)
{
	Union(aabb->min);
	Union(aabb->max);
}

bool AABB::IntersectPlane(float plane, int axis)
{
	if((aabb->min[axis] <= plane) && (aabb->max[axis] >= plane))
		return true;
	else
		return false;
}

bool AABB::ToLeft(float plane, int axis)
{
	if(aabb->max[axis] < plane)
		return true;
	else
		return false;
}

bool AABB::ToRight(float plane, int axis)
{
	return !ToLeft(plane, axis);
}


float AABB::Area()
{
	float xside = fabs(aabb->max[0] - aabb->min[0]);
	float yside = fabs(aabb->max[1] - aabb->min[1]);
	float zside = fabs(aabb->max[2] - aabb->min[2]);
	
	return xside * yside * zside;
}

void AABB::AreaLeftRight(float plane, int axis, float *aleft, float *aright)
{
	float side[3];

	side[0] = fabs(aabb->max[0] - aabb->min[0]);
	side[1] = fabs(aabb->max[1] - aabb->min[1]);
	side[2] = fabs(aabb->max[2] - aabb->min[2]);

	// Left
	side[axis] = fabs(plane - aabb->min[axis]);	
	*aleft = side[0] * side[1] * side[2];

	// Right
	side[axis] = fabs(aabb->max[axis] - plane);	
	*aright = side[0] * side[1] * side[2];	
}

void AABB::Print()
{
	cout << "min (" << aabb->min[0] << ", " << aabb->min[1] << ", " << aabb->min[2] << ")" << endl;
	cout << "max (" << aabb->max[0] << ", " << aabb->max[1] << ", " << aabb->max[2] << ")" << endl;
}

bool AABB::Intersect(ray_t *ray, float *tmin, float *tmax)
{
	float t0 = ray->tmin;
	float t1 = ray->tmax;

	for(int axis=0; axis < 3; axis++)
	{
		float invRayDir = 1.0f / ray->d[axis];
	
		float tNear = (aabb->min[axis] - ray->o[axis]) * invRayDir;
		float tFar =  (aabb->max[axis] - ray->o[axis]) * invRayDir;

		if(tNear > tFar)
		{
			float tmp = tNear;
			tNear = tFar;
			tFar = tmp;
		}

		t0 = tNear > t0 ? tNear : t0;
		t1 = tFar < t1 ? tFar : t1;


		if(t0 > t1) return false;
	}
	
	if(tmin != 0) *tmin = t0;
	if(tmax != 0) *tmax = t1;

	return true;
}



void AABB::Intersect(float *o, float *d,  float *tmin, float *tmax, int axis)
{
	float invRayDir = 1.0f / d[axis];

	float tNear = (aabb->min[axis] - o[axis]) * invRayDir;
	float tFar =  (aabb->max[axis] - o[axis]) * invRayDir;

	if(tNear > tFar)
	{
		float tmp = tNear;
		tNear = tFar;
		tFar = tmp;
	}

	*tmin = tNear;
	*tmax = tFar;
}







