#ifndef _CELL_BUFFER_H_
#define _CELL_BUFFER_H_

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#ifdef _CELL
#include <malloc_align.h>
#include <free_align.h>
#endif


typedef struct
{
	unsigned char *buffer;
	int elemsize;
	int index;
	int size;
} buffer_t;


#ifndef __SPU__
static void BufferReset(buffer_t *buf)
{
	buf->buffer = 0;
	buf->elemsize = 0;
	buf->index = 0;
	buf->size = 0;
}


static void BufferFree(buffer_t *buf)
{
	if(buf->size > 0)
	{
		#ifdef _CELL
			_free_align(buf->buffer);
		#else
			free(buf->buffer);
		#endif

		buf->size = 0;
	}
}


static void BufferCreate(buffer_t *buf, int elemsize, int size)
{
	if(size == buf->size)
	{
		buf->index = 0;
		return;
	}

	if(buf->size > 0)
	{
		#ifdef _CELL
			_free_align(buf->buffer);
		#else
			free(buf->buffer);
		#endif
	}

	#ifdef _CELL			
		buf->buffer = (unsigned char*)_malloc_align(elemsize * size, 7);
	#else
		buf->buffer = (unsigned char*)malloc(size * elemsize);
	#endif

	buf->size = size;
	buf->elemsize = elemsize;
	buf->index = 0;
}
#endif


static void BufferClear(buffer_t *buf)
{
	buf->index = 0;
}

static int BufferNumElements(buffer_t *buf)
{
	return buf->index;
}

#ifndef __SPU__
static void* BufferCopyTo(buffer_t *buf, void* data, int size)
{
	memcpy(&buf->buffer[ buf->index * buf->elemsize ], data, size * buf->elemsize);
	void *ret = &buf->buffer[ buf->index * buf->elemsize ];

	buf->index += size;

	return ret;
}
#endif 

static int BufferEmpty(buffer_t *buf)
{
	if( buf->index == 0)
		return 1;
	else
		return 0;
}

static void* BufferAllocate(buffer_t *buf, int size)
{
	void *ret = &buf->buffer[ buf->index * buf->elemsize ];

	buf->index += size;

	return ret;		
}

#endif

