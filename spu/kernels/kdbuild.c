#include <stdio.h>
#include <kdbuild.h>
#include <spu_intrinsics.h>
#include <spu_mfcio.h>
#include <string.h>
#include <dma.h>
#include <doublebuf.h>
#include <math.h>
#include <ppecallbacks.h>
#include <share/cell-buffer.h>
#include <share/kdbuffer.h>

typedef unsigned int uint;
typedef unsigned short ushort;

kdbuild_arg_t arg ALIGNED(16);

#define NUM_AABBS	((64*1024)/sizeof(aabb_t))

aabb_t aabbbuffer[2][NUM_AABBS] ALIGNED(16);
struct mfc_list_element list[2][16] ALIGNED(16);

doublebuf_t aabb_db;


bin_t minbins[2][512] ALIGNED(16);
bin_t maxbins[2][512] ALIGNED(16);

minmaxbin_t lmmb ALIGNED(16);
minmaxbin_t rmmb ALIGNED(16);

int numbins[2] = { 0, 0 };

unsigned int ABS_MASK	= 0x7FFFFFFF;
vector float abs_mask;

int nsamplepoints = 16;
int nsplitaxises = 3;
int splitaxis = 0;
int total_leaf_size ALIGNED(16);
int curjob = 0;

#define NUM_MEMCPY_TAGS	8

int cur_tag = 0;
int memcpy_tag[NUM_MEMCPY_TAGS];
int memcpy_tag_ppe[2];
int curleaf;
uint jobtag;
int numleafpolys ALIGNED(16);


void init_spu_abs()
{
	abs_mask[0] = *(float*)&ABS_MASK;
	abs_mask[1] = *(float*)&ABS_MASK;
	abs_mask[2] = *(float*)&ABS_MASK;
	abs_mask[3] = *(float*)&ABS_MASK;
}

inline vector float spu_max(vector float a, vector float b)
{
	return spu_sel( b, a, spu_cmpgt( a, b ) );
}

inline vector float spu_min(vector float a, vector float b)
{
	return spu_sel( a, b, spu_cmpgt( a, b ) );
}


inline vector float spu_abs(vector float v)
{
	return spu_and(v, abs_mask);
}

void ResetMinMaxBin(minmaxbin_t *mmb, int nbins, int index)
{
	numbins[index] = nbins;

	int i;
	vector float zero = spu_splats(0.0f);
	vector float *vminbins = (vector float*)minbins[index];
	vector float *vmaxbins = (vector float*)maxbins[index];

	for(i=0; i < numbins[index]; i++)
	{
		vminbins[i] = zero;
		vmaxbins[i] = zero;		
	}

	mmb->minbins = minbins[index];
	mmb->maxbins = maxbins[index];

	mmb->numbins = numbins[index];
	mmb->bestcost = 1000000.f;
}

inline int GetBin(float left, float right, float pos, int nbins)
{
	if(pos >= right) pos = right; 
	if(pos <= left)  pos = left;

	float width = fabs(right-left);
	float delta = width/(nbins);

	float bin = fabs(pos-left) / delta;
	
	int ibin = (int)bin;

	if(ibin > (nbins-1)) ibin = nbins-1;
	if(ibin < 0) ibin = 0;

	return ibin;
}

void MinMaxBinCount(aabb_t *aabb, minmaxbin_t *mmb, aabb_t *baabb, int a)
{
	int minindex = GetBin(baabb->min[a], baabb->max[a], aabb->min[a], nsamplepoints);
	int maxindex = GetBin(baabb->min[a], baabb->max[a], aabb->max[a], nsamplepoints);

	mmb->minbins[minindex].b[a]++;
	mmb->maxbins[maxindex].b[a]++;
}



vector signed int GetBinSIMD(vector float left, vector float right, vector float pos, vector float invdelta, vector float nbins)
{
	pos = spu_min(pos, right);
	pos = spu_max(pos, left);
	
	vector float bin = spu_mul(spu_abs(spu_sub(pos, left)), invdelta);

	bin = spu_min(bin, nbins);
	bin = spu_max(spu_splats(0.0f), bin);

	return spu_convts(bin, 0);
}

inline void MinMaxBinCount3SIMD(aabb_t *aabb, minmaxbin_t *mmb, aabb_t *baabb)
{
	vector float *baabb_min = (vector float*)baabb->min;
	vector float *baabb_max = (vector float*)baabb->max;
	vector float *aabb_min = (vector float*)aabb->min;
	vector float *aabb_max = (vector float*)aabb->max;

	vector float nbins = spu_splats((float)nsamplepoints);
	vector float invnbins = spu_re(nbins);
	nbins = spu_sub(nbins, spu_splats(1.0f));

	vector float width = spu_abs(spu_sub(*baabb_max, *baabb_min));
	vector float invdelta = spu_re(spu_mul(width, invnbins));

	vector signed int minindex = GetBinSIMD(*baabb_min, *baabb_max, *aabb_min, invdelta, nbins);
	vector signed int maxindex = GetBinSIMD(*baabb_min, *baabb_max, *aabb_max, invdelta, nbins);

	mmb->minbins[minindex[0]].b[0]++;
	mmb->minbins[minindex[1]].b[1]++;
	mmb->minbins[minindex[2]].b[2]++;

	mmb->maxbins[maxindex[0]].b[0]++;
	mmb->maxbins[maxindex[1]].b[1]++;
	mmb->maxbins[maxindex[2]].b[2]++;
}

void MinMaxBinCountAll(aabb_t *aabb, minmaxbin_t *mmb, aabb_t *baabb, int axis, int numaxises)
{
	int a;

	if(numaxises == 3 && arg.simd == 1)
	{
		MinMaxBinCount3SIMD(aabb, mmb, baabb);
	}
	else
		for(a = 0; a < numaxises; a++)
			MinMaxBinCount(aabb, mmb, baabb,  (axis+a) % 3);	
}


static float SAHCost(float area, float ctravers, float cleft, float aleft, float cright, float aright)
{
	return ctravers + (cleft * (aleft/area)) + (cright * (aright/area));
}

float Area(aabb_t *aabb)
{
	float xside = fabs(aabb->max[0] - aabb->min[0]);
	float yside = fabs(aabb->max[1] - aabb->min[1]);
	float zside = fabs(aabb->max[2] - aabb->min[2]);
	
	return xside * yside * zside;
}


void AreaLeftRight(aabb_t *aabb, float plane, int axis, float *aleft, float *aright)
{
	float side[3];

	side[0] = fabs(aabb->max[0] - aabb->min[0]);
	side[1] = fabs(aabb->max[1] - aabb->min[1]);
	side[2] = fabs(aabb->max[2] - aabb->min[2]);

	// Left
	side[axis] = fabs(plane - aabb->min[axis]);	
	*aleft = side[0] * side[1] * side[2];

	// Right
	side[axis] = fabs(aabb->max[axis] - plane);	
	*aright = side[0] * side[1] * side[2];	
}

void MinMaxBinFindBest(minmaxbin_t *mmb, kdbuffer_t *result, int a)
{
	int i;

	for(i=1; i < mmb->numbins; i++)
	{
		int j = mmb->numbins - i - 1;

		mmb->minbins[i].b[a] += mmb->minbins[i-1].b[a];
		mmb->maxbins[j].b[a] += mmb->maxbins[j+1].b[a]; 
	}

	float width = fabs(result->baabb.max[a] - result->baabb.min[a]);
	float delta = width/(mmb->numbins);
	float x = result->baabb.min[a] + delta;

	for(i=0; i < mmb->numbins-1; i++)
	{
		float aleft, aright;

		AreaLeftRight(&result->baabb, x, a, &aleft, &aright);

		float cost = SAHCost(Area(&result->baabb), 2, mmb->minbins[i].b[a], aleft, mmb->maxbins[i+1].b[a], aright);

		if(cost < mmb->bestcost)
		{
			result->plane = x;
			result->axis = a;
			result->left_size = (int)mmb->minbins[i].b[a];
			result->right_size = (int)mmb->maxbins[i+1].b[a];

			mmb->bestcost = cost;
		}

		x += delta;	
	}


}

inline void AreaLeftRightSIMD(vector float aabb_min, vector float aabb_max, vector float side, vector float plane, vector float *aleft, vector float *aright)
{
	vector float lside;
	vector float rside;

	lside = spu_abs(spu_sub(plane, aabb_min));
	rside = spu_abs(spu_sub(aabb_max, plane));

	*aleft = spu_mul(lside, side);
	*aright = spu_mul(rside, side);
}


inline vector float SAHCostSIMD(vector float invarea, vector float ctravers, vector float cleft, vector float aleft, vector float cright, vector float aright)
{
	vector float l = spu_mul(cleft, spu_mul(aleft, invarea));
	vector float r = spu_mul(cright, spu_mul(aright, invarea));

	return spu_add(ctravers, spu_add(l, r));
}



void MinMaxBinFindBest3SIMD(minmaxbin_t *mmb, kdbuffer_t *result)
{
	int i;

	for(i=1; i < mmb->numbins; i++)
	{
		int j = mmb->numbins - i - 1;

		vector float *min = (vector float *)mmb->minbins[i].b;
		vector float *max = (vector float *)mmb->maxbins[j].b;

		min[0] = spu_add(min[0], min[-1]);
		max[0] = spu_add(max[0], max[1]);
	}

	vector float *vmax = (vector float*)result->baabb.max;
	vector float *vmin = (vector float*)result->baabb.min;

	vector float vwidth = spu_abs( spu_sub(*vmax, *vmin) );

	vector float vnumbins = spu_splats(1/(float)mmb->numbins);
	vector float vdelta = spu_mul(vwidth, vnumbins);
	vector float vx = spu_add(*vmin, vdelta);

	vector float vside = { vwidth[1] * vwidth[2], vwidth[0] * vwidth[2], vwidth[0] * vwidth[1], 0 };
	vector float invarea = spu_splats( 1/(vwidth[0] * vside[0]));
	vector float vctravers = spu_splats(2.0f);
	vector float vbestcost = spu_splats(mmb->bestcost);
	vector signed int vbesti = spu_splats(0);
	vector float vbestx = vx;

	for(i=0; i < mmb->numbins-1; i++)
	{
		vector float aleft, aright;

		AreaLeftRightSIMD(*vmin, *vmax, vside, vx, &aleft, &aright);

		vector float *vminbin = (vector float *)mmb->minbins[i].b;
		vector float *vmaxbin = (vector float *)mmb->maxbins[i+1].b;

		vector float cost = SAHCostSIMD(invarea, vctravers, *vminbin, aleft, *vmaxbin, aright);

		vector unsigned int cmp = spu_cmpgt(cost, vbestcost);
		vbestcost = spu_sel(cost, vbestcost, cmp);
		vbesti = spu_sel(spu_splats(i), vbesti, cmp);
		vbestx = spu_sel(vx, vbestx, cmp);

		vx = spu_add(vx, vdelta);	
	}	

	int axis = 0;
	float bestcost = vbestcost[axis];

	if(vbestcost[1] < bestcost)
	{
		axis = 1;
		bestcost = vbestcost[1];
	}

	if(vbestcost[2] < bestcost)
	{
		axis = 2;
		bestcost = vbestcost[2];
	}

	int index = vbesti[axis];

	result->plane = vbestx[axis];
	result->axis = axis;
	result->left_size = (int)mmb->minbins[ index ].b[axis];
	result->right_size = (int)mmb->maxbins[ index+1 ].b[axis];
	
	mmb->bestcost = vbestcost[axis];
}

void MinMaxBinFindBestAll(minmaxbin_t *mmb, kdbuffer_t *result, int axis, int numaxises)
{
	int a;

	if(numaxises == 3 && arg.simd == 1)
	{
		MinMaxBinFindBest3SIMD(mmb, result);
	}
	else
		for(a = 0; a < numaxises; a++)
			MinMaxBinFindBest(mmb, result, (axis+a) % 3);	
}

void memcpy_ea(void *ea_low, void *src, int size)
{
	DmaWait(memcpy_tag[cur_tag]);
	spu_mfcdma32(src, (uint)ea_low, size, memcpy_tag[cur_tag], MFC_PUT_CMD);
	DmaWait(memcpy_tag[cur_tag]);	

	cur_tag = (cur_tag + 1) % NUM_MEMCPY_TAGS;	
}

void memcpy_ls(void *dest, void *ea_low, int size)
{
	DmaWait(memcpy_tag[cur_tag]);
	spu_mfcdma32(dest, (uint)ea_low, size, memcpy_tag[cur_tag], MFC_GET_CMD);
	DmaWait(memcpy_tag[cur_tag]);	

	cur_tag = (cur_tag + 1) % NUM_MEMCPY_TAGS;	
	
}



void MoveAABB(aabb_t *aabb, void *ea_low)
{
	memcpy_ea(ea_low, aabb, sizeof(aabb_t));	
}

void KDPartitionBegin(kdbuffer_t *in, kdbuffer_t *left, kdbuffer_t *right)
{
	// Compute left and right AABB
	vector float *src = (vector float*)&in->baabb;
	vector float *dst1 = (vector float*)&left->baabb;
	vector float *dst2 = (vector float*)&right->baabb;

	dst1[0] = src[0];
	dst1[1] = src[1];
	dst2[0] = src[0];
	dst2[1] = src[1];

	left->baabb.max[ in->axis ] = in->plane;
	right->baabb.min[ in->axis ] = in->plane;

	ResetMinMaxBin(&lmmb, nsamplepoints, 0);
	ResetMinMaxBin(&rmmb, nsamplepoints, 1);

	left->count = 0;
	right->count = 0;	

	splitaxis = (in->axis+1) % 3;
}

void KDPartition(aabb_t *aabb, int size, int axis, float plane, kdbuffer_t *left, kdbuffer_t *right)
{
	// Partition and count MinMaxBin
	int i;
	for(i=0; i < size; i++)
	{
		float min = aabb[i].min[axis];
		float max = aabb[i].max[axis];
	
		if(min < plane)
		{
			// left
			MoveAABB(&aabb[i], &left->aabb[left->count++]); 
			MinMaxBinCountAll(&aabb[i], &lmmb, &left->baabb, splitaxis, nsplitaxises);
		}

		if(max > plane)
		{
			// right
			MoveAABB(&aabb[i], &right->aabb[right->count++]);
			MinMaxBinCountAll(&aabb[i], &rmmb, &right->baabb, splitaxis, nsplitaxises);
		}
	}	
}

void KDPartitionEnd(kdbuffer_t *in, kdbuffer_t *left, kdbuffer_t *right)
{
	if(left->count > in->left_size+50)
	{
		printf("SPU KDBuild Fatal Error left count > size+50: %i > %i\n", left->count, in->left_size+50);
	}

	if(right->count > in->right_size+50)
	{
		printf("SPU KDBuild Fatal Error right count > size+50: %i > %i\n", right->count, in->right_size+50);
	}

	left->size = left->count;
	right->size = right->count;
	left->depth = in->depth+1;
	right->depth = in->depth+1;

	
	MinMaxBinFindBestAll(&lmmb, left, splitaxis, nsplitaxises);
	MinMaxBinFindBestAll(&rmmb, right, splitaxis, nsplitaxises);	
}


void KDPartitionAll(kdbuffer_t *in, kdbuffer_t *left, kdbuffer_t *right)
{
	int size[2];

	DoubleBufResetEA(&aabb_db, in->size, in->aabb);

	KDPartitionBegin(in, left, right);

	size[0] = DoubleBufGet(&aabb_db, 0);
	size[1] = DoubleBufGet(&aabb_db, 1);

	while(!DoubleBufEmpty(&aabb_db) )
	{
		DoubleBufWait(&aabb_db, 0);
		KDPartition(aabbbuffer[0], size[0], in->axis, in->plane, left, right);
		size[0] = DoubleBufGet(&aabb_db, 0);

		DoubleBufWait(&aabb_db, 1);
		KDPartition(aabbbuffer[1], size[1], in->axis, in->plane, left, right);
		size[1] = DoubleBufGet(&aabb_db, 1);
	}

	DoubleBufWait(&aabb_db, 0);
	KDPartition(aabbbuffer[0], size[0], in->axis, in->plane, left, right);
	
	DoubleBufWait(&aabb_db, 1);
	KDPartition(aabbbuffer[1], size[1], in->axis, in->plane, left, right);

	KDPartitionEnd(in, left, right);
}

unsigned char buffer[256] ALIGNED(16);



void memcpy_ppe(void *dst, void *src, int size)
{
	ushort b = 0;

	unsigned char *pdst = (unsigned char*)dst;
	unsigned char *psrc = (unsigned char*)src;

	while(size > 0)
	{
		int s = (size < 256) ? size : 256;

		DmaWait(memcpy_tag_ppe[b]);
		spu_mfcdma32(buffer, (uint)psrc, s, memcpy_tag_ppe[b], MFC_GET_CMD);
		DmaWait(memcpy_tag_ppe[b]);
		spu_mfcdma32(buffer, (uint)pdst, s, memcpy_tag_ppe[b], MFC_PUT_CMD);
		DmaWait(memcpy_tag_ppe[b]);		

		pdst += s;
		psrc += s;

		size -= s;
		b = 1 - b;
	}

	DmaWait(memcpy_tag_ppe[0]);
	DmaWait(memcpy_tag_ppe[1]);	

}

void* BufferCopyTo(buffer_t *buf, void* data, int size)
{
	if(size == 0) return &buf->buffer[ buf->index * buf->elemsize ];


	memcpy_ppe(&buf->buffer[ buf->index * buf->elemsize ], data, size*buf->elemsize);

	/*
	unsigned char *pdata = (unsigned char*)data;
	ushort i;

	for(i=0; i < size; i++)
	{
		memcpy_ls(buffer, pdata, buf->elemsize);
		memcpy_ea(&buf->buffer[ (buf->index+i) * buf->elemsize ], buffer, buf->elemsize);

		pdata += buf->elemsize;
	}
	*/

	void *ret = &buf->buffer[ buf->index * buf->elemsize ];

	buf->index += size;

	return ret;
}


void* BufferCopyToLS(buffer_t *buf, void* data, int size)
{
	memcpy_ea(&buf->buffer[ buf->index * buf->elemsize ], data, buf->elemsize * size);

	void *ret = &buf->buffer[ buf->index * buf->elemsize ];

	buf->index += size;

	return ret;
}


void MakeNodes()
{
	uint put_tag[2];

	put_tag[0] = mfc_tag_reserve();
	put_tag[1] = mfc_tag_reserve();

	ushort b = 0;

	kdbuffer_t l_kdb ALIGNED(16);
	kdbuffer_t r_kdb ALIGNED(16);

	kdnode_t node ALIGNED(16);
	kdbuffer_t kdb	ALIGNED(16);
	DoubleBufInit(&aabb_db, 0, 0, sizeof(aabb_t), NUM_AABBS, aabbbuffer[0], aabbbuffer[1]);

	// printf("Empty? %i\n", BufferEmpty(&arg.kdbuffer[b]));

	while(! BufferEmpty(&arg.kdbuffer[b]) )
	{
		kdbuffer_t *pkdb = (kdbuffer_t*)arg.kdbuffer[b].buffer;
		int size = BufferNumElements(&arg.kdbuffer[b]);
		int i;
		
		BufferClear(&arg.aabb_buffer[1-b]);
		BufferClear(&arg.kdbuffer[1-b]);
	
		// printf("size %i\n", size);

		for(i=0; i < size; i++)
		{
			l_kdb.node = arg.curnode++;
			r_kdb.node = arg.curnode++;		

			memcpy_ls(&kdb, &pkdb[i], sizeof(kdbuffer_t));

			node.split = kdb.plane;
			node.axis =  kdb.axis;
			node.left =  l_kdb.node;
			node.right = r_kdb.node;	


			memcpy_ea(&arg.nodes[ kdb.node ], &node, sizeof(kdnode_t));


			KDBufferAllocate(&l_kdb, kdb.left_size, &arg.aabb_buffer[1-b]);

			if(curjob < arg.njobs)
				KDBufferAllocate(&r_kdb, kdb.right_size, &arg.job_aabb_buffer[curjob]);
			else
				KDBufferAllocate(&r_kdb, kdb.right_size, &arg.aabb_buffer[1-b]);

			KDPartitionAll(&kdb, &l_kdb, &r_kdb);

			if(l_kdb.depth == arg.maxdepth || l_kdb.size <= arg.maxleafsize)
			{
				total_leaf_size += l_kdb.count;

				l_kdb.aabb = (aabb_t*)BufferCopyTo(&arg.leaf_aabb_buffer, l_kdb.aabb, l_kdb.count);
				BufferCopyToLS(&arg.leafbuffer, &l_kdb, 1);
			}			
			else
			{	
				BufferCopyToLS(&arg.kdbuffer[1-b], &l_kdb, 1);
			}

			if(r_kdb.depth == arg.maxdepth || r_kdb.size <= arg.maxleafsize)
			{
				total_leaf_size += r_kdb.count;
		
				if(curjob < arg.njobs)
				{
					r_kdb.aabb = (aabb_t*)BufferCopyTo(&arg.job_leaf_aabb_buffer[curjob], r_kdb.aabb, r_kdb.count);
					BufferCopyToLS(&arg.job_leafbuffer[curjob], &r_kdb, 1);

					spu_mfcdma32(&arg.job_leafbuffer[curjob], (uint)arg.pjob_leafbuffer[curjob], sizeof(buffer_t), jobtag, MFC_PUT_CMD);					
					DmaWait(jobtag);
		
				}
				else
				{
					r_kdb.aabb = (aabb_t*)BufferCopyTo(&arg.leaf_aabb_buffer, r_kdb.aabb, r_kdb.count);
					BufferCopyToLS(&arg.leafbuffer, &r_kdb, 1);

				}

			}
			else
			{
				if(curjob < arg.njobs)
				{
					BufferCopyToLS(&arg.job_kdbuffer[curjob], &r_kdb, 1);

					spu_mfcdma32(&arg.job_kdbuffer[curjob], (uint)arg.pjob_kdbuffer[curjob], sizeof(buffer_t), jobtag, MFC_PUT_CMD);					
					DmaWait(jobtag);

				}
				else
					BufferCopyToLS(&arg.kdbuffer[1-b], &r_kdb, 1);
			}


			/*
			if(curjob < njobs)
				KDBufferAllocate(&r_kdb, kdb[i].right_size, &jobs[curjob]->aabb_buffer[0]);
			else
				KDBufferAllocate(&r_kdb, kdb[i].right_size, &aabb_buffer[1-b]);

			KDPartition(&kdb[i], &l_kdb, &r_kdb);

			if(l_kdb.depth == maxdepth || l_kdb.size <= maxleafsize)
			{
				l_kdb.aabb = (aabb_t*)BufferCopyTo(&leaf_aabb_buffer, l_kdb.aabb, l_kdb.count);
				BufferCopyTo(&leafbuffer, &l_kdb, 1);
			}			
			else
				BufferCopyTo(&kdbuffer[1-b], &l_kdb, 1);

			if(r_kdb.depth == maxdepth || r_kdb.size <= maxleafsize)
			{
				if(curjob < njobs)
				{
					r_kdb.aabb = (aabb_t*)BufferCopyTo(&jobs[curjob]->leaf_aabb_buffer, r_kdb.aabb, r_kdb.count);
					BufferCopyTo(&jobs[curjob]->leafbuffer, &r_kdb, 1);
				}
				else
				{
					r_kdb.aabb = (aabb_t*)BufferCopyTo(&leaf_aabb_buffer, r_kdb.aabb, r_kdb.count);
					BufferCopyTo(&leafbuffer, &r_kdb, 1);
				}
			}
			else
			{
				if(curjob < njobs)
					BufferCopyTo(&jobs[curjob]->kdbuffer[0], &r_kdb, 1);
				else
					BufferCopyTo(&kdbuffer[1-b], &r_kdb, 1);
			}
			*/



			if(curjob < arg.njobs)
			{
				// Start other job
				
				ppe_post_sema(arg.sema[curjob]);
				curjob++;
			}


	
		}
	
	
		b =  1 - b;

	}

	while( curjob < arg.njobs)
	{
		ppe_post_sema(arg.sema[curjob]);
		curjob++;
	}



	// Transfer back
	spu_mfcdma32(&arg.curnode, (unsigned int)arg.pcurnode, (unsigned int)sizeof(int), put_tag[0], MFC_PUT_CMD);
	spu_mfcdma32(&total_leaf_size, (unsigned int)arg.ptotal_leaf_size, (unsigned int)sizeof(int), put_tag[1], MFC_PUT_CMD);
	

	DmaWait(put_tag[0]);
	DmaWait(put_tag[1]);


	spu_mfcdma32(&arg.leafbuffer, (unsigned int)arg.pleafbuffer, (unsigned int)sizeof(buffer_t), put_tag[0], MFC_PUT_CMD);
	spu_mfcdma32(&arg.leaf_aabb_buffer, (unsigned int)arg.pleaf_aabb_buffer, (unsigned int)sizeof(buffer_t), put_tag[1], MFC_PUT_CMD);

	DmaWaitAll();

	mfc_tag_release(put_tag[0]);
	mfc_tag_release(put_tag[1]);

}

int leaves[4] ALIGNED(16);

void MakeLeaf(kdbuffer_t *kdb)
{
	int n = kdb->node;

	if(curleaf > 0 && (curleaf % 4) == 0)
	{	
		memcpy_ea(&arg.pleaves[curleaf-4], &leaves, sizeof(int[4]));
	}

	leaves[ curleaf % 4 ] = n;

	kdnode_t node ALIGNED(16);

	node.left = KD_LEAF;
	node.leafid = curleaf;

	curleaf++;
	
	node.numpolys = kdb->size;
	numleafpolys += kdb->size;


	if(kdb->size > 0)
	{
		node.polys = (kdpoly_t*)BufferAllocate(&arg.polybuffer, node.numpolys);

		memcpy_ea(&arg.nodes[n], &node, sizeof(kdnode_t));
	
		int i;

		aabb_t a ALIGNED(16);
		kdpolyp_t poly ALIGNED(16);


		for(i=0; i < kdb->size; i++)
		{						
			memcpy_ls(&a, &kdb->aabb[i], sizeof(aabb_t));
			memcpy_ls(&poly, a.poly, sizeof(kdpolyp_t));

			memcpy_ppe(&node.polys[i].triangle, poly.triangle, sizeof(triangle_t));
			memcpy_ppe(&node.polys[i].normal, poly.normal, sizeof(normal_t));
			memcpy_ppe(&node.polys[i].color, poly.color, sizeof(float[4]));
		}
	}	
	else
	{
		memcpy_ea(&arg.nodes[n], &node, sizeof(kdnode_t));
	}
}


void MakeLeaves()
{
	// Make leaves
	kdbuffer_t *pkdb = (kdbuffer_t*)arg.leafbuffer.buffer;
	int size = BufferNumElements(&arg.leafbuffer);

	int i;

	kdbuffer_t kdb ALIGNED(16);

	// printf("thread %i leaves %i\n", arg.tid, size);

	for(i=0; i < size; i++)
	{
		memcpy_ls(&kdb, &pkdb[i], sizeof(kdbuffer_t));

		MakeLeaf(&kdb);
	}


	// Copy remaining leaf pointers
	int leaf;

	if( (curleaf%4) == 0)
		leaf = curleaf - 4;
	else
		leaf = curleaf - (curleaf%4);

	memcpy_ea(&arg.pleaves[leaf], &leaves, sizeof(int[4]));



}


void init_memcpy()
{
	int i;

	for(i=0; i < NUM_MEMCPY_TAGS; i++)
		memcpy_tag[i] = mfc_tag_reserve();

}


int main(unsigned long long spu_id __attribute__ ((unused)), unsigned long long parm)
{
	init_spu_abs();
	init_memcpy();

	uint tag_id = mfc_tag_reserve();
	jobtag = mfc_tag_reserve();

	memcpy_tag_ppe[0] = mfc_tag_reserve();
	memcpy_tag_ppe[1] = mfc_tag_reserve();

	// Transfer arg
	spu_mfcdma32(&arg, (unsigned int)parm, (unsigned int)sizeof(kdbuild_arg_t), tag_id, MFC_GET_CMD);
	DmaWait(tag_id);

	nsamplepoints = arg.nsamplepoints;
	nsplitaxises = arg.nsplitaxises;

	curleaf = arg.curleaf;
	curjob = 0;

	total_leaf_size = 0;

	MakeNodes();	

	DmaWaitAll();

	MakeLeaves();

	DmaWaitAll();

	spu_mfcdma32(&numleafpolys, (unsigned int)arg.numleafpolys, sizeof(int), tag_id, MFC_PUT_CMD);
	
	DmaWait(tag_id);	


	

	return 0;
}


