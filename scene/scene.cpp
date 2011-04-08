#include <iostream>
#include <math.h>
#include <stdio.h>
#include <lzmath.h>
#include <scene/kdbuildjob.h>
#include <scene/raytracejob.h>
#include <scene/scene.h>
#include <util/luabind.h>
#include <util/stats.h>
#include <util/timer.h>

#ifdef _CELL
	#include <spu/kernels/kdtravers.h>
#endif


#define SPLIT_SIZE	64*1024

using namespace lzmath;
using namespace std;

extern int KDGetNumNodes();
extern void KDSetNumberOfSplits(int n);
extern void KDSetNumberOfAxises(int n);
extern void KDMakeLeafPolys();
extern void PackKDTree();
extern void KDPrintLeaves();

static Scene *_scene;

void SetScene(Scene *s)
{
	_scene = s;
}

Scene* GetScene()
{
	return _scene;
}


Scene::Scene(ImageBuffer *imgbuf, bool animate, int renderer)
{
	this->imgbuf = imgbuf;

	Transform persp = Perspective(45.0f, 0.1f, 10000.0f);	
	
	camera = new Camera(persp, imgbuf->GetWidth(), imgbuf->GetHeight(), 45.0f, 0.01f, 10000.0f);

	SetScene(this);

	this->animate = animate;

	KDTreeMaxLeafSize(25);
	KDTreeExtraDepth(0);
	
	#ifdef _CELL
	if(renderer == R_CELL)
	{
		meshtransform_kernel = K_MESHTRANSFORM_SIMD_SPU;
		kdbuild_kernel = K_KDBUILD_SIMD_SPU;
		raygen_kernel = K_RAYGEN_F_SIMD_SPU;
		kdtravers_kernel = K_KDTRAVERS_SPU;	
		polypartition_kernel = K_POLYPARTITION_SPU;
		polytest_kernel = K_POLYTEST_SIMD_SPU;
	}
	else
	#endif
	{
		meshtransform_kernel = K_MESHTRANSFORM;	
		kdbuild_kernel = K_KDBUILD;	
		raygen_kernel = K_RAYGEN_FRUSTUM;
		kdtravers_kernel = K_KDTRAVERS;
		polypartition_kernel = K_POLYPARTITION;
		polytest_kernel = K_POLYTEST;
	}

	frustumdim = 4;
	frustumstep = 0;
	frustumstepsize = 0;

	numpolys = 0;

	shading = true;
	shadowrays = true;
	secondrays = true;
	raster = true;
	maxraydepth = 3;

	njobs = 0;
	nkdjobs = 0;
	nmtjobs = 0;

	SetLight(0, 0.0, 0, 1.0,1.0,1.0);

	pthread_mutex_init(&raster_mutex, 0);

}

void Scene::AddMesh(Mesh *m)
{
	if(m == 0)	
	{
		cout << "Error: Scene::AddMesh mesh is null" << endl;
		exit(0);
	}

	meshlist.push_back(m);

	numpolys += m->NumTriangles();
}

void Scene::Setup(ThreadPool *tp)
{
	
	MeshTransform(tp);
	RebuildKDTree(tp, true);

	#ifdef _CELL
		cout << "kD-Tree size/SPU Buffer size: " << (KDGetNumNodes()*100)/KD_NUMNODES << "%" << endl;
	#endif

	#ifdef _CELL
	if(KDGetNumNodes() > KD_NUMNODES)
	{
		cout << "Fatal error: kD-Tree to big to fit in SPU memory!" << endl;
		exit(0);
	}
	#endif

}

void Scene::SetLight(float px, float py, float pz, float dr, float dg, float db)
{
	light.pos[0] = px;	
	light.pos[1] = py;
	light.pos[2] = pz;

	light.diffuse[0] = dr;
	light.diffuse[1] = dg;
	light.diffuse[2] = db;
}

void Scene::KDTreeMaxLeafSize(int mls)
{
	kdtree_mls = mls;
}

void Scene::KDTreeExtraDepth(int etd)
{
	kdtree_etd = etd;
}

const char* Scene::MeshTransformString()
{
	switch(meshtransform_kernel)
	{
		case K_MESHTRANSFORM:			return "Transform";
		#ifdef _CELL
		case K_MESHTRANSFORM_SIMD:		return "Transform SIMD";
		case K_MESHTRANSFORM_SPU:		return "Transform SPU";
		case K_MESHTRANSFORM_SIMD_SPU:	return "Transform SIMD SPU";
		#endif
		default:	return "MeshTransform Unknown";
	}
}

void Scene::MeshTransformNext()
{
	meshtransform_kernel = (meshtransform_kernel +1) % K_MESHTRANSFORM_NUM;
}

const char* Scene::KDBuildString()
{
	switch(kdbuild_kernel)
	{
		case K_KDBUILD:					return "KDBuild";
		#ifdef _CELL
		case K_KDBUILD_SIMD:			return "KDBuild SIMD";
		case K_KDBUILD_SPU:				return "KDBuild SPU";
		case K_KDBUILD_SIMD_SPU:		return "KDBuild SIMD SPU";
		#endif
		default:						return "KDBuild Unknown";
	}
}

void Scene::KDBuildNext()
{
	kdbuild_kernel = (kdbuild_kernel+1) % K_KDBUILD_NUM;
}

const char* Scene::RayGenString()
{
	switch(raygen_kernel)
	{
		case K_RAYGEN_SINGLE: 			return "RayGenS";
		case K_RAYGEN_FRUSTUM:			return "RayGenF";
		#ifdef _CELL
		case K_RAYGEN_S_SIMD:			return "RayGenS SIMD";
		case K_RAYGEN_F_SIMD:			return "RayGenF SIMD";
		case K_RAYGEN_SINGLE_SPU:		return "RayGenS SPU";
		case K_RAYGEN_FRUSTUM_SPU:		return "RayGenF SPU";
		case K_RAYGEN_S_SIMD_SPU:		return "RayGenS SIMD SPU";
		case K_RAYGEN_F_SIMD_SPU:		return "RayGenF SIMD SPU";
		#endif
		default: 						return "RayGen Unknown";
	}

}

void Scene::RayGenNext()
{
	raygen_kernel = ( raygen_kernel + 1 ) % K_RAYGEN_NUM;
}

const char* Scene::KDTraversString()
{
	switch(kdtravers_kernel)
	{
		case K_KDTRAVERS:		return "KDTravers";
		#ifdef _CELL
		case K_KDTRAVERS_SPU:	return "KDTravers SPU";
		#endif
		default:				return "KDTravers Unknown";

	}
}

void Scene::KDTraversNext()
{
	kdtravers_kernel = (kdtravers_kernel + 1) % K_KDTRAVERS_NUM;
}
		
const char* Scene::PolyPartitionString()
{
	switch(polypartition_kernel)
	{
		case K_POLYPARTITION:		return "PolyPartition";
		#ifdef _CELL
		case K_POLYPARTITION_SPU:	return "PolyPartition SPU";
		#endif
		default:					return "PolyPartition Unknown";
	}
}

void Scene::PolyPartitionNext()
{
	polypartition_kernel = (polypartition_kernel+1) % K_POLYPARTITION_NUM;
}


const char* Scene::PolyTestString()
{
	switch(polytest_kernel)
	{
		case K_POLYTEST:			return "PolyTest";
		#ifdef _CELL
		case K_POLYTEST_SIMD:		return "PolyTest SIMD";
		case K_POLYTEST_SPU:		return "PolyTest SPU";
		case K_POLYTEST_SIMD_SPU:	return "PolyTest SIMD SPU";
		#endif
		default:			return "PolyTest Unknown";
	}
}

void Scene::PolyTestNext()
{
	polytest_kernel = ( polytest_kernel + 1 ) % K_POLYTEST_NUM;
}



void Scene::Raster(Buffer<raystate_t> &buffer_raster)
{		
	//pthread_mutex_lock(&raster_mutex);

	if(raster)
	{
		for(int i=0; i < buffer_raster.NumElements(); i++)
		{
				int x = (int)buffer_raster.buffer[i].x; 
				int y = (int)buffer_raster.buffer[i].y;

				float r = buffer_raster.buffer[i].color.c[0];
				float g = buffer_raster.buffer[i].color.c[1];
				float b = buffer_raster.buffer[i].color.c[2];

				if(buffer_raster.buffer[i].phit & RAY_SHADOW_HIT)
				{
					r *= 0.5f;
					g *= 0.5f;
					b *= 0.5f;
				}

				imgbuf->PutPixel(x,y, r, g, b);		
		}
	}


	// pthread_mutex_unlock(&raster_mutex);
}

void Scene::FrustumIncDim()
{
	if( (imgbuf->GetWidth() % frustumdim*2) == 0 && (imgbuf->GetHeight() % frustumdim*2 ) == 0)
		if( (imgbuf->GetWidth() > frustumdim*2) && (imgbuf->GetHeight() > frustumdim*2 ))
			frustumdim *= 2;
}


void Scene::FrustumDecDim()
{
	if(frustumdim > 2 )
		frustumdim /= 2;

	frustumstep = 0;
	frustumstepsize = 0;
}


void Scene::FrustumIncNumSteps()
{
	int flog = (int)log2(frustumdim);

	
	if( frustumstepsize * (frustumstep+1) <= flog)
			frustumstep++;
}

void Scene::FrustumDecNumSteps()
{
	if(frustumstep > 0)
		frustumstep--;
}

void Scene::FrustumIncStepsize()
{
	int flog = (int)log2(frustumdim);

	if( (frustumstepsize+1) * frustumstep <= flog)
			frustumstepsize++;
}

void Scene::FrustumDecStepsize()
{
	if(frustumstepsize > 0)
		frustumstepsize--;
}


void Scene::FrustumSetDim(int dim)
{
	frustumdim = dim;
}

void Scene::FrustumSetNumSteps(int steps)
{
	frustumstep = steps;
}

void Scene::FrustumSetStepSize(int stepsize)
{
	frustumstepsize = stepsize;
}

void Scene::KDSetNumSplits(int n)
{
	KDSetNumberOfSplits(n);
}

void Scene::KDSetNumAxises(int n)
{
	KDSetNumberOfAxises(n);
}

void Scene::ShadowRays(bool s)
{
	shadowrays = s;
}


void Scene::SecondRays(bool s)
{
	secondrays = s;
}

void Scene::RebuildKDTree(ThreadPool *tp, bool force)
{
	if(animate || force) 
	{
		timer.Mark();
		MeshTransform(tp);
		transform_time = timer.GetElapsed();

		timer.Mark();
		KDBuild(tp);
		kdbuild_time = timer.GetElapsed();

		KDMakeLeafPolys();

		// KDPrintLeaves();

	}
}

void Scene::InitRayTraceJobs(int njobs)
{
	if(njobs != this->njobs)
	{
		if(this->njobs > 0)
		{
			delete [] rtjob;
		}

		int maxjobs = imgbuf->GetHeight() / frustumdim;

		if(njobs > maxjobs) njobs = maxjobs;

		this->njobs = njobs;
		rtjob = new RayTraceJob[ njobs ];

		int h = imgbuf->GetHeight() / njobs;
		int y = 0;

		for(int i=0; i < njobs; i++)
		{
			rtjob[i].Init(0, y, imgbuf->GetWidth(), y+h );
			y+=h;
		}
	}	
}

void Scene::MeshTransform(ThreadPool *tp)
{

	if(nmtjobs != tp->numthreads)
	{
		if(nmtjobs > 0) delete [] mtjob;

		mtjob = new MeshTransformJob[tp->numthreads];
		nmtjobs = tp->numthreads;
	}

	for(int i=0; i < nmtjobs; i++)
	{
		mtjob[i].Init(i, nmtjobs);	
		tp->Add(&mtjob[i]);
	}

	tp->Wait();
}

void InitKDJobs(KDBuildJob *jobs, int j, int nextj, int njobs, int depth = 0)
{
	if(njobs > 1)
	{
		//cout << "depth " <<  depth << " job " << j << " left " << j << " right " << nextj << endl;

		jobs[j].AddJob( &jobs[nextj] );

		int left, right;

		if(njobs & 1)
		{
			left = (njobs/2)+1;
			right = (njobs/2);
		}
		else
		{
			left = njobs/2;
			right = njobs/2;
		}

		InitKDJobs(jobs, j, nextj+1, left, depth+1);
		InitKDJobs(jobs, nextj, nextj+left, right, depth+1);
	}


}

void Scene::KDBuild(ThreadPool *tp)
{

	if(nkdjobs != tp->numthreads)
	{
		if(nkdjobs > 0)
		{
			delete [] kdjob;
		}

		kdjob = new KDBuildJob[tp->numthreads];
		nkdjobs = tp->numthreads;
	}
	
	kdjob[0].Init(0, 0, -1, -1, true);
	
	InitKDJobs(kdjob, 0, 1, nkdjobs);
	
	for(int i=0; i < nkdjobs; i++)
	{
		kdjob[i].SetTid(i);
		tp->Add(&kdjob[i]);
	}


	tp->Wait();

	PackKDTree();

}


void Scene::Render(ThreadPool *tp, int njobs)
{

	//cout << "Render" << endl;

	// Reset timers	and stats
	transform_time = 0;
	kdbuild_time = 0;
	raygen_time = 0;
	kdtravers_time = 0;
	polypartition_time = 0;
	polytest_time = 0;
	raster_time = 0;
	
	nleafhits = 0;
	nleafvisits = 0;
	npolytests = 0;
	nfrustums = 0;
	nfrustumhits = 0;
	frustum_dep = 0;
	total_frustum_dep = 0;

	RebuildKDTree(tp);

	InitRayTraceJobs(njobs);

	for(int i=0; i < this->njobs; i++)
	{
		tp->Add(&rtjob[i]);
	}

	tp->Wait();

	
	// cout << "kdbuild done " << endl;

	// Stats
	for(int i=0; i < this->njobs; i++)
	{
		raygen_time += rtjob[i].raygen_time;
		kdtravers_time += rtjob[i].kdtravers_time;
		polypartition_time += rtjob[i].polypartition_time;
		polytest_time += rtjob[i].polytest_time;
		raster_time += rtjob[i].raster_time;

		nleafhits += rtjob[i].nleafhits;
		nleafvisits += rtjob[i].nleafvisits;
		npolytests += rtjob[i].npolytests;

		nfrustums += rtjob[i].nfrustums;
		nfrustumhits += rtjob[i].nfrustumhits;
		
		if(frustum_dep < rtjob[i].frustum_dep)
			frustum_dep = rtjob[i].frustum_dep;

		total_frustum_dep += rtjob[i].total_frustum_dep;
	}

	// Average stats
	raygen_time /= tp->numthreads;
	kdtravers_time /= tp->numthreads;
	polypartition_time /= tp->numthreads;
	polytest_time /= tp->numthreads;
	raster_time /= tp->numthreads;
}


