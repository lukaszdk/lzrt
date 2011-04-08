#include <stdio.h>
#include <kdtravers.h>
#include <spu_intrinsics.h>
#include <spu_mfcio.h>
#include <max_vec_float3.h>
#include <min_vec_float3.h>
#include <max_vec_float4.h>
#include <min_vec_float4.h>
#include <string.h>
#include <dma.h>
#include <doublebuf.h>
#include <polypartition.h>

typedef unsigned int uint;

volatile kdtravers_arg_t arg;

#define NUM_RAYS		2*1024 / sizeof(raystate_t)

struct mfc_list_element list[16] ALIGNED(16);

kdnode_t nodes[ KD_NUMNODES ] ALIGNED(16);
raystate_t in_raybuffer[2][ NUM_RAYS ] ALIGNED(16);
raystate_t out_raybuffer[2][ NUM_RAYS ] ALIGNED(16);

count_t count[NUM_LEAVES] ALIGNED(16);

doublebuf_t ray_in_db;
doublebuf_t ray_out_db;

static kdstack_t stack[32];
int numhits;

void ResetCount()
{
	int i;

	for(i=0; i < arg.numleaves; i++)
	{
		count[i].value = 0;
		count[i].offset = 0;
		count[i].ptr = 0;
	}	
}

void LeafCount(raystate_t *ray)
{
	count[ray->leaf].value++;
}


inline vector float spu_max(vector float a, vector float b)
{
	return spu_sel( b, a, spu_cmpgt( a, b ) );
}

inline vector float spu_min(vector float a, vector float b)
{
	return spu_sel( a, b, spu_cmpgt( a, b ) );
}

inline float max(float a, float b)
{
	if(a > b)
		return a;
	else
		return b;

}

inline float min(float a, float b)
{
	if(a < b)
		return a;
	else
		return b;
}

int Intersect(volatile aabb_t *aabb, ray_t *ray, float *tmin, float *tmax)
{
	vector float vray_o = *(vector float*)ray->o;
	vector float vray_d = *(vector float*)ray->d;
	vector float vaabb_min = *(vector float*)aabb->min;
	vector float vaabb_max = *(vector float*)aabb->max;

	vector float vt0 = spu_splats(ray->tmin);
	vector float vt1 = spu_splats(ray->tmax);

	vector float invraydir = spu_re(vray_d);
	vector float vp = spu_mul(vray_o, invraydir);
	
	vector float _tNear = spu_msub( vaabb_min, invraydir, vp);
	vector float _tFar = spu_msub( vaabb_max, invraydir, vp);

	vector float tNear = spu_min(_tNear, _tFar);
	vector float tFar = spu_max(_tNear, _tFar);

	vt0 = spu_max(tNear, vt0);
	vt1 = spu_min(tFar, vt1);

	*tmin = _max_vec_float3(vt0);
	*tmax = _min_vec_float3(vt1);

	return !(*tmin > *tmax);
}

inline int IsLeaf(kdnode_t *node)
{
	return !node->left;
}

inline void FindLeaf(raystate_t *ray, kdnode_t *nodes, volatile aabb_t *worldaabb)
{
	float tmin, tmax;
	
	if(ray->leafcount == 0)
	{
		if(!Intersect(worldaabb, &ray->ray, &tmin, &tmax))
		{	
			ray->leaf = -1;
			return;
		}
		else
		{
			ray->ray.tmin = max(tmin, ray->ray.tmin);
			ray->ray.tmax = min(tmax, ray->ray.tmax);
		}
	}
	else
	{
		ray->ray.tmin = ray->org_tmin;
		ray->ray.tmax = ray->org_tmax;

		tmin = ray->org_tmin;
		tmax = ray->org_tmax;
	}
	
	int stackptr = 0;
	
	float invdir[4] ALIGNED(16);
	vector float *vinvdir = (vector float *)invdir;
	vector float vray_d = *(vector float*)ray->ray.d;

	*vinvdir = spu_re( vray_d );

	int n = ray->ep;
	int leafcount = 0;

	

	while(1)
	{
		if(__builtin_expect(IsLeaf(&nodes[n]), 0))
		{
			if(ray->leafcount == leafcount) 
			{
				ray->org_tmin = ray->ray.tmin;
				ray->org_tmax = ray->ray.tmax;

				ray->ray.tmin = tmin;
				ray->ray.tmax = tmax;

				ray->leaf = nodes[n].leafid;
				ray->leafcount++;

				return;
			}

			if(stackptr == 0) 
			{
				if(ray->mode == RAY_MODE_PRIMARY)
					ray->leaf = -1;

				if(ray->mode == RAY_MODE_SHADOW)
				{
					ray->leaf = 0;
					ray->phit |= RAY_SHADOW_MISS;
				}

				if(ray->mode == RAY_MODE_REFLECTION)
				{
					ray->leaf = 0;
					ray->phit |= RAY_REFLECTIVE_MISS;
				}


				return;
			}

			stackptr--;

			n = stack[stackptr].node;
			tmin = stack[stackptr].tmin;
			tmax = stack[stackptr].tmax;
		 
			leafcount++;
		}
		else
		{
			register float tplane = (nodes[n].split - ray->ray.o[nodes[n].axis]) * invdir[nodes[n].axis];
			int first = ray->ray.o[nodes[n].axis] <= nodes[n].split;

			int firstChild, secondChild;

			if(first)
			{	
				firstChild = nodes[n].left;
				secondChild = nodes[n].right;
			}
			else
			{
				firstChild = nodes[n].right;
				secondChild = nodes[n].left;
			}

			if(tplane > tmax || tplane < 0)
			{
				n = firstChild;
			}
			else 
			{

				if(tplane < tmin)
				{
					n = secondChild;
				}
				else
				{
					stack[stackptr].node = secondChild;
					stack[stackptr].tmin = tplane;
					stack[stackptr].tmax = tmax;
	
					stackptr++; 

					n = firstChild;
					tmax = tplane;
				}
			}
		}
	}
}

inline void memcpy16(void *dst, void *src, int qsize)
{
	vector unsigned int *vdst = (vector unsigned int*)dst;
	vector unsigned int *vsrc = (vector unsigned int*)src;

	int i=0;

	for(i=0; i < qsize; i++) vdst[i] = vsrc[i];
}


void KDTravers(raystate_t *rays, int numrays, volatile aabb_t *worldaabb, raystate_t *out)
{
	int i;

	numhits = 0;

	for(i=0; i < numrays; i++)
	{		
		FindLeaf(&rays[i], nodes, worldaabb);
		
		if(rays[i].leaf >= 0) 
		{
			LeafCount(&rays[i]);

			memcpy16(&out[numhits], &rays[i], sizeof(raystate_t)/16);
			numhits++;
		}	
	}
}

inline void Decrement(int *total, int max, int *value)
{
	if(*total <= 0)
	{
		*value = 0;
		return;
	}

	if(*total > max)
	{
		*value = max;
		*total -= max;
	}
	else
	{
		*value = *total;
		*total = 0;	
	}
}


int main(unsigned long long spu_id __attribute__ ((unused)), unsigned long long parm)
{
	uint tag_id = mfc_tag_reserve();
	uint count_tag = mfc_tag_reserve();

	// Transfer arg
	spu_mfcdma32(&arg, (unsigned int)parm, (unsigned int)sizeof(kdtravers_arg_t), tag_id, MFC_GET_CMD);
	DmaWait(tag_id);	

	// Transfer kD-tree
	DmaGet(list, nodes, (unsigned int)arg.nodes, sizeof(kdnode_t)*arg.numnodes, tag_id);
	
	int ray_size[2];
	int totalnumhits = 0;

	DoubleBufInit(&ray_in_db, arg.numrays, arg.rays, sizeof(raystate_t), NUM_RAYS, in_raybuffer[0], in_raybuffer[1]);
	DoubleBufInit(&ray_out_db, arg.numrays, arg.buffer_polytest, sizeof(raystate_t), NUM_RAYS, out_raybuffer[0], out_raybuffer[1]);

	ray_size[0] = DoubleBufGet(&ray_in_db, 0);
	ray_size[1] = DoubleBufGet(&ray_in_db, 1);


	ResetCount();

	DmaWait(tag_id);

	while(!DoubleBufEmpty(&ray_in_db))
	{
		DoubleBufWait(&ray_in_db, 0);
			
		KDTravers(in_raybuffer[0], ray_size[0], &arg.worldaabb, out_raybuffer[0]);
		totalnumhits += numhits;

		ray_size[0] = DoubleBufGet(&ray_in_db, 0);
		
		// Transfer rays back to PPE 
		DoubleBufWait(&ray_out_db, 0);
		DoubleBufPut(&ray_out_db, numhits, 0);
		
		
		DoubleBufWait(&ray_in_db, 1);
			
		KDTravers(in_raybuffer[1], ray_size[1], &arg.worldaabb, out_raybuffer[1]);
		totalnumhits += numhits;

		ray_size[1] = DoubleBufGet(&ray_in_db, 1);
		
		// Transfer rays back to PPE 
		DoubleBufWait(&ray_out_db, 1);
		DoubleBufPut(&ray_out_db, numhits, 1);
	}

	DoubleBufWait(&ray_in_db, 0);
	KDTravers(in_raybuffer[0], ray_size[0], &arg.worldaabb, out_raybuffer[0]);
	totalnumhits += numhits;

	// Transfer rays back to PPE 
	DoubleBufWait(&ray_out_db, 0);
	DoubleBufPut(&ray_out_db, numhits, 0);


	DoubleBufWait(&ray_in_db, 1);
	KDTravers(in_raybuffer[1], ray_size[1], &arg.worldaabb, out_raybuffer[1]);
	totalnumhits += numhits;

	DmaPut(list, count, (uint)arg.count, sizeof(count_t) * arg.numleaves, count_tag);

		
	// Transfer rays back to PPE 
	DoubleBufWait(&ray_out_db, 1);
	DoubleBufPut(&ray_out_db, numhits, 1);

	
	spu_mfcdma32(&totalnumhits, (uint)arg.numhits, sizeof(int), tag_id, MFC_PUT_CMD);

	DmaWaitAll();

	return 0;
}
