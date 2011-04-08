#include <scene/camera.h>
#include <iostream>
#include <string.h>

using namespace std;

Camera::Camera(Transform &proj, float width, float height, float fov, float znear, float zfar)
{
	raycache = new RayTS[(int)(width*height)];
	
	this->width = width;
	this->height = height;
	this->fov = fov;
	this->znear = znear;
	this->zfar = zfar;

	cam2screen = proj;

	Set(Point3f(0,0,0), Point3f(0,0,-1), Point3f(0,1,0));
}

void Camera::Set(const Point3f &pos, const Point3f &look, const Vector3f &up)
{
	this->pos = pos;
	this->look = look;
	this->up = up;

	cam2world = LookAt(pos, look, up).GetInverse();	

	float aspect = width / height;

	Transform raster2screen = Translate(-aspect, 1 , 0) * Scale((2*aspect)/width, -(2/height), 1.0f);

	raster2cam = cam2screen.GetInverse() * raster2screen;
}

void Camera::Move(bool firstframe, float x, float y, float z)
{
	if(x == 0 && y == 0 & z == 0 & !firstframe)
	{
		hasmoved = false;
		return;
	}

	hasmoved = true;

	timestamp++;

	if(x != 0)
	{
		pos.x += x;
		look.x += x;
	}

	if(z != 0)
	{
		pos.z += z;
		look.z += z;
	}

	cam2world = LookAt(pos, look, up).GetInverse();
}

void Camera::GenerateRay(Ray *ray, const float x, const float y) const
{
	Point3f p_raster = Point3f(x, y, 0);
	Point3f p_cam;

	raster2cam(&p_cam, p_raster);

	ray->o = Point3f(0, 0, 0);
	ray->d = Normalize(p_cam);

	ray->mint = 0.0f;
	ray->maxt = (zfar - znear);	
	
	cam2world(ray, *ray);
}

void Camera::GenerateRayCached(Ray *ray, const float x, const float y) const
{
	int r = ((int)y*(int)width) + (int)x;

	if(hasmoved || raycache[r].timestamp < timestamp)
	{
		raycache[r].timestamp = timestamp;

		Point3f p_raster = Point3f(x, y, 0);
		Point3f p_cam;

		raster2cam(&p_cam, p_raster);

		raycache[r].o = Point3f(0, 0, 0);
		raycache[r].d = Normalize(p_cam);

		raycache[r].mint = 0.0f;
		raycache[r].maxt = (zfar - znear);	

		cam2world(&raycache[r], raycache[r]);
	}	

	ray->Clone(raycache[r]);

}

void Camera::GetRaster2Cam(float matrix[4][4])
{
	memcpy(matrix, raster2cam.matrix->m, sizeof(float[4][4]));
}

void Camera::GetCam2World(float matrix[4][4])
{
	memcpy(matrix, cam2world.matrix->m, sizeof(float[4][4]));
}


void Camera::GetViewDirection(Vector3f *v)
{
	*v = Normalize(look - pos);
}







