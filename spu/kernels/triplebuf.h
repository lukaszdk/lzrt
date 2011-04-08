#ifndef _SPU_TRIPLEBUF_H_
#define _SPU_TRIPLEBUF_H_

#include <dma.h>

typedef struct
{
	int num;
	int max;
	void *buf[3];
	void *ea_low;
	int elemsize;
	int offset;
	int size[3];
	int tag_id[3];
	struct mfc_list_element list[3][16] ALIGNED(16);
} triplebuf_t;


void TripleBufInit(triplebuf_t *tb, int num, void *ea_low, int elemsize, int max, void *buf1, void *buf2, void *buf3)
{
	tb->num = num;	
	tb->max = max;
	tb->ea_low = ea_low;
	tb->buf[0] = buf1;
	tb->buf[1] = buf2;
	tb->buf[2] = buf3;
	tb->elemsize = elemsize;
	tb->offset = 0;
	tb->size[0] = 0;
	tb->size[1] = 0;
	tb->size[2] = 0;
	tb->tag_id[0] = mfc_tag_reserve();
	tb->tag_id[1] = mfc_tag_reserve();
	tb->tag_id[2] = mfc_tag_reserve();
}

inline void TripleBufResetEA(triplebuf_t *tb, int num, void *ea_low)
{
	tb->num = num;
	tb->ea_low = ea_low;
	tb->offset = 0;
}

inline void TripleBufReset(triplebuf_t *tb, int num)
{
	tb->num = num;
}

inline void TripleBufResetAll(triplebuf_t *tb, int num)
{
	tb->num = num;
	tb->offset = 0;
}

inline int TripleBufGet(triplebuf_t *tb, int buf)
{
	tb->size[buf] = (tb->num < tb->max) ? tb->num : tb->max;

	DmaGet(tb->list[buf], tb->buf[buf], ((uint)tb->ea_low)+(tb->offset*tb->elemsize), tb->size[buf] * tb->elemsize, tb->tag_id[buf]);

	tb->offset += tb->size[buf];
	tb->num -= tb->size[buf];

	return tb->size[buf];
}

inline int TripleBufDec(triplebuf_t *tb, int buf)
{
	tb->size[buf] = (tb->num < tb->max) ? tb->num : tb->max;

	return tb->size[buf];	
}

inline void TripleBufPut(triplebuf_t *tb, int tsize, int buf)
{
	DmaPut(tb->list[buf], tb->buf[buf], ((uint)tb->ea_low)+(tb->offset*tb->elemsize), tsize * tb->elemsize, tb->tag_id[buf]);

	tb->offset += tsize;
	tb->num -= tb->size[buf];	
}




inline void TripleBufWait(triplebuf_t *tb, int buf)
{
	DmaWait(tb->tag_id[buf]);
}

inline int TripleBufEmpty(triplebuf_t *tb)
{
	return (tb->num == 0);
}

#endif

