#ifndef	_SPE_POLYTEST_H_
#define	_SPE_POLYTEST_H_

#include <share/structs.h>

typedef struct
{
	polytest_t *polytests;
	int numpolytests;
	float lightpos[4] ALIGNED(16);
	float lightdiffuse[4] ALIGNED(16);
	raystate_t *buffer_kdtravers;
	raystate_t *buffer_raster;
	void *numhits;
	int simd;
	int shadowrays;
	int secondrays;
	int shading;
	int *npolytests;

} polytest_arg_t;



#endif
