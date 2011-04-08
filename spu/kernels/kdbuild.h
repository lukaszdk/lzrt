#ifndef _SPE_KDBUILD_H_
#define _SPE_KDBUILD_H_

#include <share/structs.h>
#include <share/cell-buffer.h>

#define SPU_KDBUILD_KDPARTITION		0

typedef struct
{
	int nsamplepoints;
	int nsplitaxises;
	int curnode ALIGNED(16);
	int curleaf;
	int *pcurnode;
	int *ptotal_leaf_size;
	int *pleaves;
	int *pcurleaf;
	int maxdepth;
	int maxleafsize;
	int tid;
	kdnode_t *nodes;
	int njobs;
	void *sema[8];
	buffer_t job_aabb_buffer[8] ALIGNED(16);
	buffer_t job_kdbuffer[8] ALIGNED(16);
	buffer_t job_leaf_aabb_buffer[8] ALIGNED(16);
	buffer_t job_leafbuffer[8] ALIGNED(16);
	buffer_t *pjob_leafbuffer[8];
	buffer_t *pjob_kdbuffer[8];
	int simd ALIGNED(16);
	buffer_t aabb_buffer[2] ALIGNED(16);	
	buffer_t kdbuffer[2] ALIGNED(16);
	buffer_t leaf_aabb_buffer ALIGNED(16);
	buffer_t leafbuffer ALIGNED(16);
	buffer_t polybuffer;
	buffer_t *pleaf_aabb_buffer;
	buffer_t *pleafbuffer;
	int *numleafpolys;


} kdbuild_arg_t;




#endif

