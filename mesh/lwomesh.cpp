#include <iostream>
#include <lwoloader.h>
#include <lzmath.h>
#include <mesh/aabb.h>
#include <mesh/lwomesh.h>
#include <string.h>
#include <stdlib.h>

#ifdef _CELL
	#include <malloc_align.h>
	#include <free_align.h>
#endif

using namespace std;

extern bool fileExists(const char* filename);
extern char* filePath();

LWOMesh::LWOMesh(const char *filename)
{
	Load(filename);
}

LWOMesh::LWOMesh()
{
	numtriangles = 0;
}

void LWOMesh::Load(const char *filename)
{
	mesh_t *mesh;

	if(fileExists(filename))
	{
		mesh = LoadLWO((char*)filename);		
	}
	else
	{
		char altfilename[512];
		memset(altfilename, 0, 512);

		strcat(altfilename, filePath());
		strcat(altfilename, "/");
		strcat(altfilename, filename);

		if(fileExists(altfilename))
		{
			mesh = LoadLWO((char*)altfilename);		
		}
		else
		{
			cout << "Error: Unable to load LWO model '" << filename << "'" << endl;
			return;
		}
	}

	numtriangles = mesh->numtris;

	// cout << "LWOMesh '" << filename << "' loaded (" << numtriangles << " triangles)" << endl;

	#ifdef _CELL	
		org_triangles = (triangle_t*)_malloc_align( numtriangles * sizeof(triangle_t), 4);
		triangles = (triangle_t*)_malloc_align( numtriangles * sizeof(triangle_t), 4);
		org_normals = (normal_t*)_malloc_align( numtriangles * sizeof(normal_t), 4);
		normals = (normal_t*)_malloc_align( numtriangles * sizeof(normal_t), 4);
		aabbs = (aabb_t*)_malloc_align( numtriangles * sizeof(aabb_t), 4);
		colors = (color_t*)_malloc_align( numtriangles * sizeof(color_t), 4);
	#else
		org_triangles = new triangle_t[numtriangles];
		triangles = new triangle_t[numtriangles];
		org_normals = new normal_t[numtriangles];
		normals = new normal_t[numtriangles];
		aabbs = new aabb_t[numtriangles];
		colors = new color_t[numtriangles];	
	#endif

	for(int i=0; i < numtriangles; i++)
	{
		org_triangles[i].v1[0] = triangles[i].v1[0] = mesh->tris[i].v1[0];
		org_triangles[i].v1[1] = triangles[i].v1[1] = mesh->tris[i].v1[1];
		org_triangles[i].v1[2] = triangles[i].v1[2] = mesh->tris[i].v1[2];
		org_triangles[i].v1[3] = triangles[i].v1[3] = mesh->tris[i].v1[3] = 1;

		org_triangles[i].v2[0] = triangles[i].v2[0] = mesh->tris[i].v2[0];
		org_triangles[i].v2[1] = triangles[i].v2[1] = mesh->tris[i].v2[1];
		org_triangles[i].v2[2] = triangles[i].v2[2] = mesh->tris[i].v2[2];
		org_triangles[i].v2[3] = triangles[i].v2[3] = mesh->tris[i].v2[3] = 1;

		org_triangles[i].v3[0] = triangles[i].v3[0] = mesh->tris[i].v3[0];
		org_triangles[i].v3[1] = triangles[i].v3[1] = mesh->tris[i].v3[1];
		org_triangles[i].v3[2] = triangles[i].v3[2] = mesh->tris[i].v3[2];
		org_triangles[i].v3[3] = triangles[i].v3[3] = mesh->tris[i].v3[3] = 1;

		org_normals[i].n1[0] = normals[i].n1[0] = mesh->norms[i].n1[0];
		org_normals[i].n1[1] = normals[i].n1[1] = mesh->norms[i].n1[1];
		org_normals[i].n1[2] = normals[i].n1[2] = mesh->norms[i].n1[2];
		org_normals[i].n1[3] = normals[i].n1[3] = mesh->norms[i].n1[3] = 1;

		org_normals[i].n2[0] = normals[i].n2[0] = mesh->norms[i].n2[0];
		org_normals[i].n2[1] = normals[i].n2[1] = mesh->norms[i].n2[1];
		org_normals[i].n2[2] = normals[i].n2[2] = mesh->norms[i].n2[2];
		org_normals[i].n2[3] = normals[i].n2[3] = mesh->norms[i].n2[3] = 1;

		org_normals[i].n3[0] = normals[i].n3[0] = mesh->norms[i].n3[0];
		org_normals[i].n3[1] = normals[i].n3[1] = mesh->norms[i].n3[1];
		org_normals[i].n3[2] = normals[i].n3[2] = mesh->norms[i].n3[2];
		org_normals[i].n3[3] = normals[i].n3[3] = mesh->norms[i].n3[3] = 1;

		colors[i].c[0] = mesh->c[i].c[0];
		colors[i].c[1] = mesh->c[i].c[1];
		colors[i].c[2] = mesh->c[i].c[2];
		colors[i].reflective = 0;

		AABB aabb(&aabbs[i]);		

		aabb.Reset();
		aabb.Union(triangles[i].v1);
		aabb.Union(triangles[i].v2);
		aabb.Union(triangles[i].v3);
	}

	FreeMesh(mesh);
}

LWOMesh::~LWOMesh()
{
	#ifdef _CELL	
		_free_align(org_triangles);
		_free_align(triangles);
		_free_align(org_normals);
		_free_align(normals);
		_free_align(aabbs);
		_free_align(colors);
	#else
		delete [] org_triangles;
		delete [] triangles;
		delete [] org_normals;
		delete [] normals;
		delete [] aabbs;
		delete [] colors;	
	#endif
}

void LWOMesh::DoTransform()
{
	for(int i=0; i < numtriangles; i++)
	{
		transform(triangles[i].v1, org_triangles[i].v1);
		transform(triangles[i].v2, org_triangles[i].v2);
		transform(triangles[i].v3, org_triangles[i].v3);
	
		transform.ApplyNormal(org_normals[i].n1, normals[i].n1);
		transform.ApplyNormal(org_normals[i].n2, normals[i].n2);
		transform.ApplyNormal(org_normals[i].n3, normals[i].n3);
	
		AABB aabb(&aabbs[i]);		

		aabb.Reset();
		aabb.Union(triangles[i].v1);
		aabb.Union(triangles[i].v2);
		aabb.Union(triangles[i].v3);
	}

	
}

int LWOMesh::NumTriangles()
{
	return numtriangles;
}

triangle_t* LWOMesh::GetTriangles(int *num)
{
	if(num != 0) *num = numtriangles;

	return triangles;
}
	
normal_t* LWOMesh::GetNormals(int *num)
{
	if(num != 0) *num = numtriangles;
	return normals;
}

triangle_t* LWOMesh::GetOrgTriangles(int *num)
{
	if(num != 0) *num = numtriangles;

	return org_triangles;
}
	
normal_t* LWOMesh::GetOrgNormals(int *num)
{
	if(num != 0) *num = numtriangles;
	return org_normals;
}

aabb_t* LWOMesh::GetAABBs(int *num)
{
	if(num != 0) *num = numtriangles;
	return aabbs;
}

color_t *LWOMesh::GetColors(int *num)
{
	if(num != 0) *num = numtriangles;

	return colors;
}




