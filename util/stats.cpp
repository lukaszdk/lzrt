#include <util/stats.h>
#include <iostream>


using namespace std;

static Stats stats(4);

extern bool animate;

Stats* GetStats()
{
	return &stats;
}

Stats::Stats(int nthreads)
{
	this->nthreads = nthreads;

	primtests = new Uint32[nthreads];
	leavesvisited = new Uint32[nthreads];

	fps = 0;
	Reset();
}

void Stats::Reset()
{
	rayhitwaabb = 0;
	prevfps = GetFPS();
	fps = 0;
	rebuildtime = 0;
	rendertime = 0;
	praygentime = 0;
	

	for(int i=0; i < nthreads; i++)
		primtests[i] = 0;

	for(int i=0; i < nthreads; i++)
		leavesvisited[i] = 0;
}


Uint32 Stats::AvgPrimTest()
{
	if(rayhitwaabb > 0)
	{
		Uint32 tprimtests = 0;

		for(int i=0; i < nthreads; i++) 
		{
			tprimtests += primtests[i];
		}

		return tprimtests/rayhitwaabb;
	}			
	else
		return 0;
}

float Stats::GetFPS()
{
	return (1000.0f) / (float)(rendertime);
}

float Stats::AvgLeavesPrRay()
{
	float leavesv = 0;

	for(int i=0; i < nthreads; i++)
	{
		// cout << "leavesvisited[" << i << "] = " << scene->kdtree->leavesvisited[i] << endl;

		leavesv += leavesvisited[i];
	}

	if(rayhitwaabb > 0)
		return (leavesv / (float)(rayhitwaabb));
	else
		return 0;
}

Uint32 Stats::AvgLeafSize()
{
	if(numleaves > 0)
		return totalleafsize / numleaves;
	else
		return 0;
}

float Stats::WorldAABBHitPct(ImageBuffer *imgbuf)
{
	return (float)(rayhitwaabb * 100)/(float)(imgbuf->GetHeight() * imgbuf->GetWidth());
}

float Stats::PrimCopyFactor()
{
	if(worldsize > 0)
		return (float)totalleafsize / (float)worldsize;
	else
		return 0;
}

#define NTHREADS	2

float Stats::CpuUsage()
{
	// Rebuild is only using 1 thread

	float rbt = (float)rebuildtime;
	float rdt = (float)rendertime;

	return (((rbt/NTHREADS) + (rdt - rbt))  / rdt) * 100;
}



