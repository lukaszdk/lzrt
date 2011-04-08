#include <lzmath.h>
#include <mesh/aabb.h>
#include <mesh/custommesh.h>
#include <scene/math3d.h>

#ifdef _CELL
	#include <malloc_align.h>
	#include <free_align.h>
#endif

CustomMesh::CustomMesh(int numtriangles)
{
	this->numtriangles = numtriangles;

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

	index = 0;

}

CustomMesh::~CustomMesh()
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


void CustomMesh::DoTransform()
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

void CustomMesh::BeginTriangle()
{
	vertex = 0;
	reflective = 0;
}

void CustomMesh::Vertex(float v1, float v2, float v3)
{
	switch(vertex)
	{
		default:
		case 0: 
		{
			org_triangles[index].v1[0] = v1;
			org_triangles[index].v1[1] = v2;
			org_triangles[index].v1[2] = v3;
			org_triangles[index].v1[3] = 1;

			vertex++;
		} break;

		case 1: 
		{
			org_triangles[index].v2[0] = v1;
			org_triangles[index].v2[1] = v2;
			org_triangles[index].v2[2] = v3;
			org_triangles[index].v2[3] = 1;

			vertex++;
		} break;

		case 2: 
		{
			org_triangles[index].v3[0] = v1;
			org_triangles[index].v3[1] = v2;
			org_triangles[index].v3[2] = v3;
			org_triangles[index].v3[3] = 1;

			vertex++;
		} break;		
	}

}

void CustomMesh::Color( float r, float g, float b)
{
	colors[index].c[0] = r;
	colors[index].c[1] = g;
	colors[index].c[2] = b;
}

void CustomMesh::Reflective(float r)
{
	reflective = r;
}

void CustomMesh::EndTriangle()
{
	// Compute AABB
	AABB aabb(&aabbs[index]);	

	aabb.Reset();

	aabb.Union(triangles[index].v1);
	aabb.Union(triangles[index].v2);
	aabb.Union(triangles[index].v3);

	// Compute normals (flat shading)
	float v1[4];
	float v2[4];
	float n[4];

	VectorSub(org_triangles[index].v2, org_triangles[index].v1, v1);
	VectorSub(org_triangles[index].v3, org_triangles[index].v1, v2);

	VectorCross(v1, v2, n);
	VectorNormalize(n);

	// cout << "n " << n[0] << ", " << n[1] << ", " << n[2] << endl;

	org_normals[index].n1[0] = normals[index].n1[0] = n[0];
	org_normals[index].n1[1] = normals[index].n1[1] = n[1];
	org_normals[index].n1[2] = normals[index].n1[2] = n[2];
	org_normals[index].n1[3] = normals[index].n1[3] = 1;

	org_normals[index].n2[0] = normals[index].n2[0] = n[0];
	org_normals[index].n2[1] = normals[index].n2[1] = n[1];
	org_normals[index].n2[2] = normals[index].n2[2] = n[2];
	org_normals[index].n2[3] = normals[index].n2[3] = 1;

	org_normals[index].n3[0] = normals[index].n3[0] = n[0];
	org_normals[index].n3[1] = normals[index].n3[1] = n[1];
	org_normals[index].n3[2] = normals[index].n3[2] = n[2];
	org_normals[index].n3[3] = normals[index].n3[3] = 1;

	// Copy triangles
	triangles[index].v1[0] = org_triangles[index].v1[0];
	triangles[index].v1[1] = org_triangles[index].v1[1];
	triangles[index].v1[2] = org_triangles[index].v1[2];
	triangles[index].v1[3] = org_triangles[index].v1[3];

	triangles[index].v2[0] = org_triangles[index].v2[0];
	triangles[index].v2[1] = org_triangles[index].v2[1];
	triangles[index].v2[2] = org_triangles[index].v2[2];
	triangles[index].v2[3] = org_triangles[index].v2[3];

	triangles[index].v3[0] = org_triangles[index].v3[0];
	triangles[index].v3[1] = org_triangles[index].v3[1];
	triangles[index].v3[2] = org_triangles[index].v3[2];
	triangles[index].v3[3] = org_triangles[index].v3[3];

	colors[index].reflective = reflective;

	index++;
}


int CustomMesh::NumTriangles()
{
	return numtriangles;
}

triangle_t* CustomMesh::GetTriangles(int *num)
{
	if(num != 0) *num = numtriangles;

	return triangles;
}
	
normal_t* CustomMesh::GetNormals(int *num)
{
	if(num != 0) *num = numtriangles;
	return normals;
}

triangle_t* CustomMesh::GetOrgTriangles(int *num)
{
	if(num != 0) *num = numtriangles;

	return org_triangles;
}
	
normal_t* CustomMesh::GetOrgNormals(int *num)
{
	if(num != 0) *num = numtriangles;
	return org_normals;
}

aabb_t* CustomMesh::GetAABBs(int *num)
{
	if(num != 0) *num = numtriangles;
	return aabbs;
}

color_t *CustomMesh::GetColors(int *num)
{
	if(num != 0) *num = numtriangles;

	return colors;
}
