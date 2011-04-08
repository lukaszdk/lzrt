#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <lzmath.h>

using namespace lzmath;

class RayTS : public Ray
{
	public:
		long timestamp;
	public:
		RayTS()
		{
			timestamp = 0;
		}
};

class Camera
{
	public:
		float width;
		float height;
		float fov;
		float znear;
		float zfar;
		Point3f pos;
		Point3f look;
		Vector3f up;
		bool hasmoved;
		RayTS *raycache;
		// Transform world2cam;
		Transform cam2world;
		Transform cam2screen; // Projection
		Transform raster2cam;
		long timestamp;
	public:
		Camera(Transform &proj, float width, float height, float fov, float znear, float zfar);
		void Set(const Point3f &pos, const Point3f &look, const Vector3f &up);
		void Move(bool firstframe, float x, float y, float z);

		void GenerateRay(Ray *ray, const float x, const float y) const;
		void GenerateRayCached(Ray *ray, const float x, const float y) const;
		void GetViewDirection(Vector3f *v);
		
		void GetRaster2Cam(float matrix[4][4]);
		void GetCam2World(float matrix[4][4]);



};


#endif

