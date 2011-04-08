#ifndef _STRUCTS_H_
#define _STRUCTS_H_

#ifndef __SPU__
#include <iostream>
using namespace std;
#endif

#define RAY_MODE_PRIMARY		0
#define RAY_MODE_SHADOW			1
#define RAY_MODE_REFLECTION		2

#define RAY_PRIMARY_HIT			(1 << 0)
#define RAY_REFLECTIVE_BIT		(1 << 1)
#define RAY_SHADOW_HIT			(1 << 2)
#define RAY_REFLECTIVE_HIT		(1 << 3)
#define RAY_SHADOW_MISS			(1 << 4)
#define RAY_REFLECTIVE_MISS		(1 << 5)

#define ALIGNED(n)	__attribute__ ((aligned (n)))

#define KD_LEAF		0

#define KD_AXIS_X	0
#define KD_AXIS_Y	1
#define KD_AXIS_Z	2

typedef struct
{
	float o[3] ALIGNED(16);
	float tmin;
	float d[3] ALIGNED(16);
	float tmax;
} ray_t;

typedef struct
{
	float v1[4] ALIGNED(16);
	float v2[4] ALIGNED(16);
	float v3[4] ALIGNED(16);
} triangle_t;

typedef struct
{
	float n1[4] ALIGNED(16);
	float n2[4] ALIGNED(16);
	float n3[4] ALIGNED(16);
} normal_t;

typedef struct
{
	float x,y;
} coord_t;

typedef struct
{
	float c[3] ALIGNED(16);
	float reflective;
} color_t;

typedef struct
{
	ray_t ray ALIGNED(16);	
	float hitp[4] ALIGNED(16);
	float normal[4] ALIGNED(16);
	color_t color ALIGNED(16);	
	union
	{
		color_t rcolor ALIGNED(16);
		float	org_dir[4];
	};
	float x,y;
	int phit;
	int depth;	
	int leaf;	
	int leafcount;
	int ep; // entrypoint
	int mode;
	float org_tmin;
	float org_tmax;
} raystate_t;

typedef struct
{
	triangle_t triangle ALIGNED(16);
	normal_t normal ALIGNED(16);
	float color[3] ALIGNED(16);
	float reflective;
} kdpoly_t;

typedef struct
{
	triangle_t *triangle ALIGNED(16);
	normal_t *normal;
	float *color;
} kdpolyp_t;


typedef struct
{
	union
	{
		float min[4] ALIGNED(16);
		struct
		{
			unsigned int pad[3];
			kdpolyp_t *poly;
		};
	};
	float max[4] ALIGNED(16);
} aabb_t;


typedef struct
{
	union
	{
		float split;
		kdpoly_t *polys;
	};	

	union
	{
		unsigned int axis;
		unsigned int numpolys;
	};
	
	union
	{
		unsigned int child[2];
			
		struct
		{
			unsigned int left;
			union
			{
				unsigned int right;
				unsigned int leafid;
			};
		};
	};

} kdnode_t;

typedef struct
{
	int node;
	float tmin;
	float tmax;
} kdstack_t;

typedef struct
{
	kdpoly_t *polys ALIGNED(16);
	int numpolys;
} kdleafpoly_t;

typedef struct
{
	raystate_t *rays;
	int numrays;
	kdpoly_t *polys;
	int numpolys;
} polytest_t;


typedef struct
{
	aabb_t *aabb ALIGNED(16);
	aabb_t baabb; 
	int size;
	int count;
	// Split plane
	float plane;
	int axis;
	int depth;
	int node;
	// Size of left and right buffer
	int left_size;
	int right_size;
} kdbuffer_t;

typedef struct
{
	float b[4] ALIGNED(16);
} bin_t;

typedef struct
{	
	int numbins;
	bin_t *minbins;
	bin_t *maxbins;
	float bestcost;
} minmaxbin_t;

typedef struct
{
	float o[4][4] ALIGNED(16);
	float d[4][4] ALIGNED(16);
} frustum_t;

typedef struct
{
	int value;
	int offset;
	int ptr;
	int pad;
} count_t;

#ifndef __SPU__
inline void TestStructs()
{
	#ifdef _CELL
	if(sizeof(triangle_t) % 16) 
	{
		cout << "Error: sizeof(triangle_t) = " << sizeof(triangle_t) << endl;
		exit(0);
	}

	if(sizeof(ray_t) % 16) 
	{
		cout << "Error: sizeof(ray_t) = " << sizeof(ray_t) << endl;
		exit(0);
	}

	if(sizeof(normal_t) % 16) 
	{
		cout << "Error: sizeof(normal_t) = " << sizeof(normal_t) << endl;
		exit(0);
	}

	if(sizeof(aabb_t) % 16) 
	{
		cout << "Error: sizeof(aabb_t) = " << sizeof(aabb_t) << endl;
		exit(0);
	}

	if(sizeof(color_t) % 16) 
	{
		cout << "Error: sizeof(color_t) = " << sizeof(color_t) << endl;
		exit(0);
	}

	if(sizeof(raystate_t) % 16) 
	{
		cout << "Error: sizeof(raystate_t) = " << sizeof(raystate_t) << endl;
		exit(0);
	}

	if(sizeof(kdpoly_t) % 16) 
	{
		cout << "Error: sizeof(kdpoly_t) = " << sizeof(kdpoly_t) << endl;
		exit(0);
	}

	if(sizeof(kdnode_t) % 16) 
	{
		cout << "Error: sizeof(kdnode_t) = " << sizeof(kdnode_t) << endl;
		exit(0);
	}

	if(sizeof(kdleafpoly_t) % 16) 
	{
		cout << "Error: sizeof(kdleafpoly_t) = " << sizeof(kdleafpoly_t) << endl;
		exit(0);
	}



	#endif


	// cout << "All structs OK!" << endl;
}
#endif

#endif

