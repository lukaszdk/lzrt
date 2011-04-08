#ifndef _LWOMESH_H_
#define _LWOMESH_H_

#include <mesh/mesh.h>
#include <lwoloader.h>

class LWOMesh : public Mesh
{
	public:
		int numtriangles;
		triangle_t *org_triangles;
		triangle_t *triangles;
		normal_t *normals;
		normal_t *org_normals;
		aabb_t *aabbs;
		color_t *colors;
	public:
		LWOMesh(const char *filename);
		LWOMesh();
		~LWOMesh();
		void Load(const char *filename);
		int NumTriangles();
		triangle_t *GetTriangles(int *num);
		triangle_t *GetOrgTriangles(int *num);
		normal_t *GetNormals(int *num);
		normal_t *GetOrgNormals(int *num);
		aabb_t *GetAABBs(int *num);
		color_t *GetColors(int *num);
		void DoTransform();
};

#endif


