#ifndef _MESHTRANSFORMJOB_H_
#define _MESHTRANSFORMJOB_H_

#include <share/structs.h>
#include <util/buffer.h>
#include <util/threadpool.h>
#include <util/timer.h>

class MeshTransformJob: public Job
{
	public:
		int nthreads;
		int tid;
	public:
		void Init(int tid, int nthreads);
		void Run(int thread_id);
		void MeshTransform();
		void MeshTransformSPU();
		bool Free();
};

#endif

