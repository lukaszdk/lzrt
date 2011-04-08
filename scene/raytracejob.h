#ifndef _RAYTRACE_JOB_H_
#define _RAYTRACE_JOB_H_

#include <share/structs.h>
#include <util/buffer.h>
#include <util/threadpool.h>
#include <util/timer.h>

class RayTraceJob: public Job
{
	public:
		int startx;
		int starty;
		int endx;
		int endy;
		count_t *count;
		int count_size;
		Buffer<raystate_t> buffer_kdtravers;
		Buffer<raystate_t> buffer_polypartition;
		Buffer<raystate_t> buffer_polytest;
		Buffer<raystate_t> buffer_raster;	
		Timer timer;
		unsigned int transform_time;
		unsigned int kdbuild_time;
		unsigned int raygen_time;
		unsigned int kdtravers_time;
		unsigned int polypartition_time;
		unsigned int polytest_time;
		unsigned int raster_time;
		unsigned int nleafhits;
		unsigned int nleafvisits;
		unsigned int npolytests;
		unsigned int nfrustums;
		unsigned int nfrustumhits;
		unsigned int frustum_dep;
		unsigned int total_frustum_dep;
		int tid;
	public:
		void Init(int startx, int starty, int endx, int endy);
		void Run(int thread_id);
		void ResetStats();
		void SetTid(int tid);
		bool Free();
		// Kernels
		void RayGen(int startx, int starty, int endx, int endy);
		void RayGenSingle(int startx, int starty, int endx, int endy, int ep = 0);
		void RayGenFrustumRecursive(int x, int y, int frustumdim, int frustumstep);
		void RayGenFrustum(int startx, int starty, int endx, int endy);
		#ifdef _CELL
		int FindEntryPointSIMD(frustum_t *f, kdnode_t *nodes, aabb_t *worldaabb);
		#endif
		int FindEntryPoint(frustum_t *f, kdnode_t *nodes, aabb_t *worldaabb);
		void KDTravers();
		void ResetPolyPartition();
		void PolyPartitionCount(raystate_t *ray);
		void PolyPartition(polytest_t **list, int *size); 
		void PolyTest(polytest_t *list, int size, float lpos[3], float ldiffuse[3]);
		void Raster();
		#ifdef _CELL
		// SPU Kernels	
		void RayGenSPU(int startx, int starty, int endx, int endy);
		void PolyPartitionSPU(polytest_t **list, int *size); 
		void KDTraversSPU();
		void PolyTestSPU(polytest_t *list, int size, float lpos[3], float ldiffuse[3]);
		#endif

};

void FindLeaf(raystate_t *ray, kdnode_t *nodes, aabb_t *worldaabb, int tid);

#endif


