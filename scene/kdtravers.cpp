#include <algorithm>
#include <mesh/aabb.h>
#include <scene/math3d.h>
#include <scene/raytracejob.h>
#include <scene/scene.h>
#ifdef _CELL
	#include <spu/kernels/kdtravers.h>
	#include <vec_types.h>
	#include <altivec.h>
	#include <spu2vmx.h>
	#include <max_vec_float3.h>
	#include <min_vec_float3.h>
	#include <max_vec_float4.h>
	#include <min_vec_float4.h>
#endif

#ifdef _CELL
extern spe_program_handle_t kdtravers;
#endif

extern kdnode_t* KDGetNodes(int *nnodes = 0);
extern int KDGetNumLeaves();
aabb_t *KDGetWorldAABB();

static kdstack_t stack[16][32];

static inline int IsLeaf(kdnode_t *node)
{
	return !node->left;
}

#ifdef _CELL
int IntersectSIMD(volatile aabb_t *aabb, ray_t *ray, float *tmin, float *tmax)
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


#endif


void FindLeaf(raystate_t *ray, kdnode_t *nodes, aabb_t *worldaabb, int tid)
{
	float tmin, tmax;
	
	if(ray->leafcount == 0)
	{
		#ifdef _CELL_2 // BUG
			if(!IntersectSIMD(worldaabb, &ray->ray, &tmin, &tmax))
		#else
			AABB waabb(worldaabb);
			if(!waabb.Intersect(&ray->ray, &tmin, &tmax)) 
		#endif
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
	
	#ifdef _CELL
		float invdir[4] ALIGNED(16);
		vector float *vinvdir = (vector float *)invdir;
		vector float vray_d = *(vector float*)ray->ray.d;

		*vinvdir = spu_re( vray_d );
	#else
		float invdir[4]  = { 1.0f / ray->ray.d[0], 1.0f / ray->ray.d[1], 1.0f / ray->ray.d[2] } ;
	#endif

	int n = ray->ep;
	int leafcount = 0;

	register short select[4];

	#ifdef _CELL
		vector unsigned int result;		
	#else
		unsigned int result[4];
	#endif

	while(1)
	{
		if(IsLeaf(&nodes[n]))
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

			n = stack[tid][stackptr].node;
			tmin = stack[tid][stackptr].tmin;
			tmax = stack[tid][stackptr].tmax;
		 
			leafcount++;
		}
		else
		{
			register float tplane = (nodes[n].split - ray->ray.o[nodes[n].axis]) * invdir[nodes[n].axis];
			bool first = ray->ray.o[nodes[n].axis] <= nodes[n].split;

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
					stack[tid][stackptr].node = secondChild;
					stack[tid][stackptr].tmin = tplane;
					stack[tid][stackptr].tmax = tmax;
	
					stackptr++; 

					n = firstChild;
					tmax = tplane;
				}
			}
		}
	}
}

#ifdef _CELL
void RayTraceJob::KDTraversSPU()
{
	int numhits ALIGNED(16);

	ResetPolyPartition();

	numhits = 0;

	kdtravers_arg_t arg ALIGNED(16);


	memcpy(&arg.worldaabb, KDGetWorldAABB(), sizeof(aabb_t));
	arg.nodes = KDGetNodes(&arg.numnodes);	
	
	arg.rays = (raystate_t*)buffer_kdtravers.buffer; 
	arg.numrays =  buffer_kdtravers.NumElements();
	arg.buffer_polytest = (raystate_t*)buffer_polypartition.CopyToPtr();
	arg.numhits = &numhits;
	arg.numleaves = KDGetNumLeaves();
	arg.count = count;

	// Load and Run SPE program
	SPEProgram spe_kdtravers(&kdtravers);
	spe_kdtravers.Run(GetSPEContext(tid), &arg);	

	nleafvisits += numhits;

	buffer_polypartition.Increment(numhits);
	buffer_kdtravers.Clear();

}
#endif

void RayTraceJob::KDTravers()
{
	int hitcount = 0;

	raystate_t *rays = (raystate_t*)buffer_kdtravers.buffer; 
	raystate_t *dest = (raystate_t*)buffer_polypartition.buffer;
	int numrays = buffer_kdtravers.NumElements();

	int numnodes;

	ResetPolyPartition();
	
	for(int i=0; i < numrays; i++)
	{		
		FindLeaf(&rays[i], KDGetNodes(), KDGetWorldAABB(), tid);		
	
		if(rays[i].leaf >= 0) 
		{
			PolyPartitionCount(&rays[i]);
			memcpy(&dest[hitcount], &rays[i], sizeof(raystate_t));
			hitcount++;
		}	
	}

	nleafvisits += hitcount;

	buffer_polypartition.Increment(hitcount);
	buffer_kdtravers.Clear();

}

