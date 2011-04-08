#ifndef _SPE_RAYGEN_H_
#define _SPE_RAYGEN_H_

#include <share/structs.h>

typedef struct
{
	vector float raster2cam[4];
	vector float cam2world[4];
	int startx, starty;
	int endx, endy;
	unsigned int numrays;
	void *buffer_polypartition;
	float znear, zfar;
	int simd ALIGNED(16);
	int frustum;
	int frustumdim;
	int frustumstepsize;
	int frustumstep;
	kdnode_t *nodes;
	int numnodes;
	int *numhits;
	aabb_t worldaabb;
	int numleaves;
	count_t *count;
	unsigned int *nfrustums;
	unsigned int *nfrustumhits;
	unsigned int *frustum_dep;
	unsigned int *total_frustum_dep;
} raygen_arg_t;


#endif


