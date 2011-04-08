#ifndef _KDBUFFER_H_
#define _KDBUFFER_H_

#include <share/cell-buffer.h>

#ifndef __SPU__
void KDBufferAllocate(kdbuffer_t *kdb, int bsize)
{
	// +50 to fix problems with buffer sizes computed differently
	// in minmaxbin and partition due to numeric precision of floats.

	#ifdef _CELL	
		kdb->aabb = (aabb_t*)_malloc_align((bsize+50)*sizeof(aabb_t), 7); 
	#else
		kdb->aabb = new aabb_t[bsize+50]; 
	#endif

	kdb->size = bsize;
}


void KDBufferAllocate(kdbuffer_t *kdb, int bsize, Buffer<aabb_t> *buf)
{
	// +50 to fix problems with buffer sizes computed differently
	// in minmaxbin and partition due to numeric precision of floats.

	kdb->aabb = buf->Allocate(bsize+50); 
	kdb->size = bsize;
}
#endif

void KDBufferAllocate(kdbuffer_t *kdb, int bsize, buffer_t *buf)
{
	// +50 to fix problems with buffer sizes computed differently
	// in minmaxbin and partition due to numeric precision of floats.

	kdb->aabb = (aabb_t*)BufferAllocate(buf, bsize+50); 
	kdb->size = bsize;
}

void KDBufferFree(kdbuffer_t *kdb)
{
	#ifdef _CELL
		_free_align(kdb->aabb);		
	#else
		delete [] kdb->aabb;
	#endif
}


#endif

