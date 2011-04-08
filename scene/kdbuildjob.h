#ifndef _KDBUILD_JOB_H_
#define _KDBUILD_JOB_H_

#include <share/cell-buffer.h>
#include <share/structs.h>
#include <util/buffer.h>
#include <util/threadpool.h>
#include <util/timer.h>
#include <stdlib.h>

class KDBuildJob: public Job
{
	public:
		buffer_t *aabb_buffer[2] ALIGNED(16);
		buffer_t *kdbuffer[2] ALIGNED(16);
		buffer_t *leaf_aabb_buffer ALIGNED(16);
		buffer_t *leafbuffer ALIGNED(16);	
		buffer_t *polybuffer ALIGNED(16);
		bool loadpolys;
		int curnode ALIGNED(16);
		int curleaf ALIGNED(16);
		int startnode;
		int startleaf;
		int endnode;
		int endleaf;
		bool root;
		sem_t sem;
		KDBuildJob *jobs[10];
		int njobs;
		int curjob;
		int tid;
		int numbins[2];;
		bin_t *minbins[2];
		bin_t *maxbins[2];
		int splitaxis;
	public:
		KDBuildJob();
		~KDBuildJob();
		void Init(int startnode, int startleaf, int endnode = -1, int endleaf = -1, bool root = false);
		void Run(int thread_id);
		void SetTid(int tid);
		bool Free();
		void AddJob(KDBuildJob *j);
		void MeshTransform();
		void MakeLeaf(kdbuffer_t *kdb);	
		void ResetMinMaxBin(minmaxbin_t *mmb, int nbins, int index);
		void KDPartition(kdbuffer_t *in, kdbuffer_t *left, kdbuffer_t *right);
		void KDBuild(int maxleafsize, int polycopyfactor, bool root = false);
		void MeshTransformSPU();
		void KDBuildSPU(int maxleafsize, int polycopyfactor, bool root = false);
};


#endif

