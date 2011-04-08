#ifndef _MESH_H_
#define _MESH_H_

#include <lzmath.h>
#include <lwoloader.h>
#include <share/structs.h>


using namespace lzmath;

class Mesh
{
	public:
		Transform transform;
	public:
		virtual triangle_t *GetTriangles(int *num = 0) = 0;
		virtual triangle_t *GetOrgTriangles(int *num  = 0) = 0;
		virtual normal_t *GetNormals(int *num  = 0) = 0;
		virtual normal_t *GetOrgNormals(int *num  = 0) = 0;
		virtual aabb_t *GetAABBs(int *num = 0) = 0;
		virtual color_t *GetColors(int *num = 0) = 0;
		virtual int NumTriangles() = 0;
		virtual void DoTransform() = 0;

		void Apply(Transform t)
		{	
			transform = t;
		}
};

#endif

