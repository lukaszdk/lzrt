#ifndef _SCENE_H_
#define _SCENE_H_

#include <SDL.h>
#include <vector>

#include <imagebuffer/imagebuffer.h>
#include <mesh/custommesh.h>
#include <mesh/mesh.h>
#include <mesh/lwomesh.h>
#include <scene/camera.h>
#include <scene/kdbuildjob.h>
#include <scene/meshtransformjob.h>
#include <scene/raytracejob.h>
#include <util/luabind.h>
#include <util/buffer.h>
#include <util/threadpool.h>
#include <util/timer.h>
#include <share/cell-buffer.h>
#include <pthread.h>

#ifdef _CELL
	#include <spu/speprogram.h>
#endif

#ifdef _CELL
	#define	K_MESHTRANSFORM				0
	#define K_MESHTRANSFORM_SIMD		1
	#define K_MESHTRANSFORM_SPU			2
	#define K_MESHTRANSFORM_SIMD_SPU	3
	#define K_MESHTRANSFORM_NUM			4
	#define K_SPU_MESHTRANSFORM			2
#else
	#define	K_MESHTRANSFORM		0
	#define K_MESHTRANSFORM_NUM	1
#endif

#ifdef _CELL
	#define K_KDBUILD					0
	#define K_KDBUILD_SIMD				1
	#define K_KDBUILD_SPU				2
	#define K_KDBUILD_SIMD_SPU			3
	#define K_KDBUILD_NUM				4
	#define K_SPU_KDBUILD				2
#else
	#define K_KDBUILD					0
	#define K_KDBUILD_NUM				1
#endif

#ifdef _CELL
	#define	K_RAYGEN_SINGLE			0
	#define K_RAYGEN_FRUSTUM		1
	#define K_RAYGEN_S_SIMD			2
	#define K_RAYGEN_F_SIMD			3
	#define K_RAYGEN_SINGLE_SPU		4
	#define K_RAYGEN_FRUSTUM_SPU	5
	#define K_RAYGEN_S_SIMD_SPU		6
	#define K_RAYGEN_F_SIMD_SPU		7
	#define K_RAYGEN_NUM			8
	#define K_SPU_RAYGEN			4

#else
	#define	K_RAYGEN_SINGLE		0
	#define K_RAYGEN_FRUSTUM	1
	#define K_RAYGEN_NUM		2
#endif

#ifdef _CELL
	#define K_KDTRAVERS			0
	#define K_KDTRAVERS_SPU		1
	#define	K_KDTRAVERS_NUM		2
	#define K_SPU_KDTRAVERS		1
#else
	#define K_KDTRAVERS			0
	#define K_KDTRAVERS_NUM		1
#endif

#ifdef _CELL
	#define K_POLYPARTITION		0
	#define K_POLYPARTITION_SPU	1
	#define K_POLYPARTITION_NUM	2
	#define K_SPU_POLYPARTITION	1
#else
	#define K_POLYPARTITION		0
	#define K_POLYPARTITION_NUM	1
#endif

#ifdef _CELL
	#define K_POLYTEST			0
	#define K_POLYTEST_SIMD		1
	#define K_POLYTEST_SPU		2
	#define K_POLYTEST_SIMD_SPU	3
	#define K_POLYTEST_NUM		4
	#define K_SPU_POLYTEST		2
#else
	#define K_POLYTEST			0
	#define K_POLYTEST_NUM		1
#endif

#ifdef _CELL
	#define R_GENERIC	0
	#define R_CELL		1
	#define R_NUM		2
#else
	#define R_GENERIC	0
	#define R_NUM		1
#endif



using namespace std;

typedef struct
{
	float pos[4] ALIGNED(16);
	float diffuse[4];
} light_t;

class Scene
{
	public:
		Camera *camera;
		int screen_width;
		int screen_height;
		int kdtree_mls;
		int kdtree_etd;
		ImageBuffer *imgbuf;
		bool usekdtree;
		bool shadowrays;
		bool shading;
		bool secondrays;
		bool raster;
		int maxraydepth;
		bool frustum;
		bool animate;
		int meshtransform_kernel;
		int kdbuild_kernel;
		int raygen_kernel;
		int kdtravers_kernel;
		int polypartition_kernel;
		int polytest_kernel;
		int frustumdim;
		int frustumstep;
		int frustumstepsize;
		vector<Mesh*> meshlist;
		int numpolys;
		Timer timer;
		pthread_mutex_t raster_mutex;
		int njobs;
		RayTraceJob *rtjob;
		KDBuildJob *kdjob;
		int nkdjobs;
		int	nmtjobs;
		MeshTransformJob *mtjob;
		light_t light ALIGNED(16);
		unsigned int transform_time;
		unsigned int kdbuild_time;
		unsigned int raygen_time;
		unsigned int kdtravers_time;
		unsigned int polypartition_time;
		unsigned int polytest_time;
		unsigned int raster_time;
		// Stats
		unsigned int nleafhits;
		unsigned int nleafvisits;
		unsigned int npolytests;
		unsigned int nfrustums;
		unsigned int nfrustumhits;
		unsigned int frustum_dep;
		unsigned int total_frustum_dep;
	public:
		Scene(ImageBuffer *imgbuf, bool animate = true, int renderer = R_GENERIC);		
		void Setup(ThreadPool *tp);	
		void AddMesh(Mesh *m);
		void SetLight(float px, float py, float pz, float dr, float dg, float db);
		void AddLWOMesh(LWOMesh *m) { AddMesh(m); }
		void AddCustomMesh(CustomMesh *m) { AddMesh(m); }
		void KDTreeMaxLeafSize(int mls);
		void KDTreeExtraDepth(int etd);
		void FrustumSetDim(int dim);
		void FrustumSetNumSteps(int steps);
		void FrustumSetStepSize(int stepsize);
		void FrustumIncDim();
		void FrustumDecDim();
		void FrustumIncNumSteps();
		void FrustumDecNumSteps();
		void FrustumIncStepsize();
		void FrustumDecStepsize();
		void KDSetNumSplits(int n);
		void KDSetNumAxises(int n);
		void ShadowRays(bool s);
		void SecondRays(bool s);
		// Kernel string and switching
		const char* MeshTransformString();
		void MeshTransformNext();
		const char* KDBuildString();
		void KDBuildNext();
		const char* RayGenString();
		void RayGenNext();
		const char* KDTraversString();
		void KDTraversNext();
		const char* PolyPartitionString();
		void PolyPartitionNext();
		const char* PolyTestString();
		void PolyTestNext();
		// Render functions
		void InitRayTraceJobs(int njobs);
		void Render(ThreadPool *tp, int njobs);

		void MeshTransform(ThreadPool *tp);
		void KDBuild(ThreadPool *tp);
		void RebuildKDTree(ThreadPool *tp, bool force = false);	
		void KDInitBuild(int maxleafsize, int polycopyfactor, bool loadpolys = true);
		void KDLoadMeshes(KDBuildJob *job, buffer_t *partitionbuffer, buffer_t *leafbuffer, int maxleafsize, int *curnode);		
		void Raster(Buffer<raystate_t> &buffer_raster);
};

void SetScene(Scene *s);
Scene* GetScene();


#endif


