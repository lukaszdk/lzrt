#ifndef _SPE_KDTRAVERS_H_
#define _SPE_KDTRAVERS_H_

#include <share/structs.h>

#define KD_NUMNODES		((150*1024) / sizeof(kdnode_t))

typedef struct
{
	kdnode_t *nodes;
	int numnodes;
	raystate_t *rays;
	unsigned int numrays;
	raystate_t *buffer_polytest;
	int *numhits;
	aabb_t worldaabb ALIGNED(16);
	int numleaves;
	count_t *count;

} kdtravers_arg_t;

#endif

