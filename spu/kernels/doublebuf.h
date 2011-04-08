#ifndef _SPU_DOUBLEBUF_H_
#define _SPU_DOUBLEBUF_H_

#include <dma.h>

typedef struct
{
	int num;
	int max;
	void *buf[2];
	void *ea_low;
	int elemsize;
	int offset;
	int size[2];
	int tag_id[2];
	struct mfc_list_element list[2][16] ALIGNED(16);
} doublebuf_t;


void DoubleBufInit(doublebuf_t *db, int num, void *ea_low, int elemsize, int max, void *buf1, void* buf2)
{
	db->num = num;	
	db->max = max;
	db->ea_low = ea_low;
	db->buf[0] = buf1;
	db->buf[1] = buf2;
	db->elemsize = elemsize;
	db->offset = 0;
	db->size[0] = 0;
	db->size[1] = 0;
	db->tag_id[0] = mfc_tag_reserve();
	db->tag_id[1] = mfc_tag_reserve();

}

inline void DoubleBufResetEA(doublebuf_t *db, int num, void *ea_low)
{
	db->num = num;
	db->ea_low = ea_low;
	db->offset = 0;
}

inline void DoubleBufReset(doublebuf_t *db, int num)
{
	db->num = num;
}

inline void DoubleBufResetAll(doublebuf_t *db, int num)
{
	db->num = num;
	db->offset = 0;
}

inline int DoubleBufGet(doublebuf_t *db, int buf)
{
	db->size[buf] = (db->num < db->max) ? db->num : db->max;

	DmaGet(db->list[buf], db->buf[buf], ((uint)db->ea_low)+(db->offset*db->elemsize), db->size[buf] * db->elemsize, db->tag_id[buf]);

	db->offset += db->size[buf];
	db->num -= db->size[buf];

	return db->size[buf];
}

inline int DoubleBufDec(doublebuf_t *db, int buf)
{
	db->size[buf] = (db->num < db->max) ? db->num : db->max;

	return db->size[buf];	
}

inline void DoubleBufPut(doublebuf_t *db, int tsize, int buf)
{
	DmaPut(db->list[buf], db->buf[buf], ((uint)db->ea_low)+(db->offset*db->elemsize), tsize * db->elemsize, db->tag_id[buf]);

	db->offset += tsize;
	db->num -= db->size[buf];	
}




inline void DoubleBufWait(doublebuf_t *db, int buf)
{
	DmaWait(db->tag_id[buf]);
}

inline int DoubleBufEmpty(doublebuf_t *db)
{
	return (db->num == 0);
}

#endif

