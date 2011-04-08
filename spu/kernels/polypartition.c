#include <stdio.h>
#include <polypartition.h>
#include <spu_intrinsics.h>
#include <spu_mfcio.h>
#include <malloc_align.h>
#include <free_align.h>
#include <dma.h>
#include <doublebuf.h>

#define NUM_RAYS	124 //(int)((16*1024)/sizeof(raystate_t))

typedef unsigned int uint;

polypartition_arg_t arg ALIGNED(16);
kdleafpoly_t leafpolys[NUM_LEAVES] ALIGNED(16);
count_t count[NUM_LEAVES] ALIGNED(16);
polytest_t list[NUM_LEAVES] ALIGNED(16);

raystate_t raybuffer[2][NUM_RAYS] ALIGNED(16);
struct mfc_list_element getlist[16] ALIGNED(16);
struct mfc_list_element putlist[16] ALIGNED(16);
struct mfc_list_element countlist[16] ALIGNED(16);

doublebuf_t ray_db;

#define NUM_TAGS	4
int tags[NUM_TAGS];

inline int ComputeOffsets()
{
	int i=0;

	int size = 0;

	if(count[0].value > 0) size = 1;

	// Compute offsets and pointers
	for(i=1; i < arg.numleaves; i++)
	{
		if(count[i].value > 0) size++;

		count[i].offset = count[i-1].offset + count[i-1].value;
		count[i].ptr = count[i].offset;
	}

	return size;
}

inline void MoveRays(raystate_t *rays, int numrays)
{
	int i;
	int cur_id = 0;

	for(i=0; i < numrays; i++)
	{
		int leaf = rays[i].leaf;
	
		DmaWait(tags[cur_id]);
		spu_mfcdma32(&rays[i], (uint)&arg.dest[count[leaf].ptr++], sizeof(raystate_t), tags[cur_id], MFC_PUT_CMD);



		cur_id = (cur_id + 1) % NUM_TAGS;	
	}
}

void WaitRays()
{
	int i;

	for(i=0; i < NUM_TAGS; i++)
		DmaWait(tags[i]);
}


void ComputeAndTransferList(int size, int tagid)
{
	int j = 0;
	int i;

	int asize ALIGNED(16);

	asize = size;

	for(i=0; i < arg.numleaves; i++)
	{
		if(count[i].value > 0)
		{
			polytest_t *pp = &list[j];

			pp->rays = &arg.dest[count[i].offset];
			pp->numrays = count[i].value;
			pp->polys = leafpolys[i].polys;
			pp->numpolys = leafpolys[i].numpolys;		
			
			j++;
		}
	}

	spu_mfcdma32(&asize, (uint)arg.listsize, sizeof(int), tagid, MFC_PUT_CMD);
	DmaWait(tagid);	

	DmaPut(putlist, list, (uint)arg.list, size * sizeof(polytest_t), tagid);
	//DmaWait(tagid);
}

int main(unsigned long long spu_id __attribute__ ((unused)), unsigned long long parm)
{
	uint tag_id;

	tag_id = mfc_tag_reserve();
	uint count_tag = mfc_tag_reserve();

	int i;

	for(i=0; i < NUM_TAGS; i++)
		tags[i] = mfc_tag_reserve();

	// Transfer arg
	spu_mfcdma32(&arg, parm, sizeof(polypartition_arg_t), tag_id, MFC_GET_CMD);
	DmaWait(tag_id);	

	// Transfer kdleafpolys
	DmaGet(getlist, leafpolys, (uint)arg.leafpolys, sizeof(kdleafpoly_t) * arg.numleaves, tag_id);

	// Transfer count
	DmaGet(countlist, count, (uint)arg.count, sizeof(count_t) * arg.numleaves, count_tag);

	int raysize[2];

	// printf("num_rays %i\n", NUM_RAYS);
	
	DoubleBufInit(&ray_db, arg.numrays, arg.rays, sizeof(raystate_t), NUM_RAYS, raybuffer[0], raybuffer[1]);

	raysize[0] = DoubleBufGet(&ray_db, 0);
	raysize[1] = DoubleBufGet(&ray_db, 1);	

	DmaWait(tag_id);
	DmaWait(count_tag);

	int size = ComputeOffsets();
	ComputeAndTransferList(size, tag_id);

	while(!DoubleBufEmpty(&ray_db))
	{
		DoubleBufWait(&ray_db, 0);
		MoveRays(raybuffer[0], raysize[0]);

		raysize[0] = DoubleBufGet(&ray_db, 0);

		DoubleBufWait(&ray_db, 1);
		MoveRays(raybuffer[1], raysize[1]);

		raysize[1] = DoubleBufGet(&ray_db, 1);
	}
	
	DoubleBufWait(&ray_db, 0);
	MoveRays(raybuffer[0], raysize[0]);

	DoubleBufWait(&ray_db, 1);
	MoveRays(raybuffer[1], raysize[1]);

	DmaWaitAll();


 	return 0;
}
