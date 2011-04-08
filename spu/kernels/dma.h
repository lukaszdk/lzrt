#ifndef _SPE_DMA_H_
#define _SPE_DMA_H_

#include <spu_intrinsics.h>
#include <spu_mfcio.h>

void DmaGet(struct mfc_list_element *list, void *buffer, unsigned int ea_low, int size, unsigned int tagid)
{
	if(size <= 0) return;

	int listsize = 0;
	int i = 0;

	while(size > 0)
	{
		list[i].size = (size < 16384) ? size : 16384;
		list[i].eal = ea_low;
		list[i].notify = 0;		
		
		ea_low += list[i].size;
		size -= list[i].size;		
		i++;
	}

	listsize = i * sizeof(struct mfc_list_element);

	spu_mfcdma32(buffer, (unsigned int)list, listsize, tagid, MFC_GETL_CMD);
}

void DmaPut(struct mfc_list_element *list, void *buffer, unsigned int ea_low, int size, unsigned int tagid)
{
	if(size <= 0) return;

	int listsize = 0;
	int i = 0;

	while(size > 0)
	{
		list[i].size = (size < 16384) ? size : 16384;
		list[i].eal = ea_low;
		list[i].notify = 0;		
		
		ea_low += list[i].size;
		size -= list[i].size;		
		i++;
	}

	listsize = i * sizeof(struct mfc_list_element);

	spu_mfcdma32(buffer, (unsigned int)list, listsize, tagid, MFC_PUTL_CMD);
}

inline void DmaWait(unsigned int tagid)
{
	mfc_write_tag_mask(1<<tagid);
	mfc_read_tag_status_all();
}

inline void DmaWaitAll()
{
	mfc_write_tag_mask(-1);
	mfc_read_tag_status_all();
}


#endif


