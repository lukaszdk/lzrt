#ifndef _SPE_MESHTRANSFORM_H_
#define _SPE_MESHTRANSFORM_H_

#include <share/structs.h>

typedef struct
{
	vector float matrix[4];
	vector float invmatrix[4];
	triangle_t *t_src;
	triangle_t *t_dest;
	aabb_t *aabb;
	normal_t *n_src;
	normal_t *n_dest;
	int numtriangles;
	int simd;

} meshtransform_arg_t;


#endif


