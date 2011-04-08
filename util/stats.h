#ifndef _STATS_H_
#define _STATS_H_

#include <SDL.h>
#include <imagebuffer/imagebuffer.h>

class Stats
{
	public:
		int nthreads;
		float fps;
		float prevfps;
		Uint32 rayhitwaabb;
		Uint32 rebuildtime;
		Uint32 rendertime;
		Uint32 praygentime;
		Uint32 *primtests;
		Uint32 *leavesvisited;
		Uint32 totalleafsize;
		Uint32 numleaves;
		Uint32 worldsize;
		Uint32 transformtime;
	public:
		Stats(int nthreads);
		void Reset();
		float GetFPS();
		Uint32 AvgPrimTest();
		float AvgLeavesPrRay();
		Uint32 AvgLeafSize();
		float WorldAABBHitPct(ImageBuffer *imgbuf);	
		float PrimCopyFactor();
		float CpuUsage();



};

Stats* GetStats();

#endif

