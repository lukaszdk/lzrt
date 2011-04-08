#include <lzray.h>

using namespace lzmath;

Ray::Ray()
{
	mint = LZRAY_EPSILON;
	maxt = LZRAY_INFINITY;
}

void Ray::Clone(Ray &ray)
{
	o.x = ray.o.x;
	o.y = ray.o.y;
	o.z = ray.o.z;
	
	d.x = ray.d.x;
	d.y = ray.d.y;
	d.z = ray.d.z;

	mint = ray.mint;
	maxt = ray.maxt;
}

Ray::Ray(const Point3f &origin, const Vector3f &direction, float start, float end)
{
	o = origin;
	d = direction;
	mint = start;
	maxt = end;
}


Point3f Ray::operator()(float t) const
{
	return o + (d* t);
}

void Ray::Reset()
{
	mint = LZRAY_EPSILON;
	maxt = LZRAY_INFINITY;
}

