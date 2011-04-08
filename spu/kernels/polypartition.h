#ifndef _SPE_POLYPARTITION_H_
#define _SPE_POLYPARTITION_H_

#include <share/structs.h>

#define NUM_LEAVES	((64*1024)/sizeof(kdleafpoly_t))

typedef struct
{
	kdleafpoly_t *leafpolys ALIGNED(16);
	int numleaves;
	raystate_t *rays;
	raystate_t *dest;
	int numrays;
	polytest_t *list;
	int *listsize;
	count_t *count;

} polypartition_arg_t;

#endif

