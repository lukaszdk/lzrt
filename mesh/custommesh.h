#ifndef _CUSTOMMESH_H_
#define _CUSTOMMESH_H_

#include <mesh/mesh.h>

class CustomMesh : public Mesh
{
	public:
		int numtriangles;
		int index;
		int vertex;
		triangle_t *org_triangles;
		triangle_t *triangles;
		normal_t *normals;
		normal_t *org_normals;
		aabb_t *aabbs;
		color_t *colors;
		float reflective;
	public:
		CustomMesh(int numtriangles);
		~CustomMesh();
		void BeginTriangle();
		void Vertex(float v1, float v2, float v3);
		void Color( float r, float g, float b);
		void EndTriangle();
		void Reflective(float r);
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


