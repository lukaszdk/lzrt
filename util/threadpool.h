#ifndef _THREADPOOL_H_
#define _THREADPOOL_H_

#include <semaphore.h>
#include <pthread.h>
#include <list>
#include <util/timer.h>

class Job
{
	public:
		virtual void Run(int thread_id) = 0;
		virtual bool Free() = 0;
};

class ThreadPool
{
	public:
		int numthreads;
		sem_t availablejobs;
		sem_t idlethreads;
		sem_t runningthreads;
		pthread_t *pthreads;
		pthread_mutex_t mutex;
		std::list<Job*> jobs;
		bool run;
		Timer timer;
	public:
		ThreadPool(int numthreads);
		void Init(int numthreads);
		void SetNumThreads(int numthreads);
		void Add(Job *job);
		void Wait();
		void Exit();
};


#endif
