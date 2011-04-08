#include <scene/raytracejob.h>
#include <scene/scene.h>

using namespace std;

void RayTraceJob::Init(int startx, int starty, int endx, int endy)
{
	this->startx = startx;
	this->starty = starty;
	this->endx = endx;
	this->endy = endy;

	count_size = 0;

	ResetStats();
}

void RayTraceJob::ResetStats()
{
	// Reset timers
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

}

void RayTraceJob::SetTid(int tid)
{
	this->tid = tid;
}

void RayTraceJob::Run(int thread_id)
{
	int size = (endx-startx) * (endy-starty);

	ResetStats();

	buffer_kdtravers.Create(size);
	buffer_polypartition.Create(size);
	buffer_polytest.Create(size);
	buffer_raster.Create(size);

	Scene *scene = GetScene();

	tid = thread_id;

	// RayGen
	timer.Mark();	

	#ifdef _CELL
		if(GetScene()->raygen_kernel >= K_SPU_RAYGEN)
			RayGenSPU(startx, starty, endx, endy);
		else
	#endif
	
	RayGen(startx, starty, endx, endy);
	raygen_time = timer.GetElapsed();

	bool first = true;

	// cout << "New " <<  endl;

	do
	{
		// KDTravers
		timer.Mark();

		if(!first)
		{
			#ifdef _CELL
				if(GetScene()->kdtravers_kernel >= K_SPU_KDTRAVERS)
					KDTraversSPU();
				else
			#endif
		
			KDTravers();
		}

		kdtravers_time += timer.GetElapsed();

		// PolyPartition
		timer.Mark();

		polytest_t *list;
		int size;

		#ifdef _CELL
			if(GetScene()->polypartition_kernel >= K_SPU_POLYPARTITION)
				PolyPartitionSPU(&list, &size);
			else
		#endif
			PolyPartition(&list, &size);

		polypartition_time += timer.GetElapsed();
		
		// PolyTest
		timer.Mark();
		#ifdef _CELL
			if(GetScene()->polytest_kernel >= K_SPU_POLYTEST)
				PolyTestSPU(list, size, scene->light.pos, scene->light.diffuse);
			else
		#endif
		
		PolyTest(list, size, scene->light.pos, scene->light.diffuse);
		polytest_time += timer.GetElapsed();

		// cout << "npolytests " << npolytests << endl;


		first = false;
	
	} while( ! buffer_kdtravers.IsEmpty() );


	// Raster
	timer.Mark();
	scene->Raster(buffer_raster);
	raster_time = timer.GetElapsed();
}

bool RayTraceJob::Free()
{
	return false;
}

