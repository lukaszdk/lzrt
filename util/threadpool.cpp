#include <util/threadpool.h>
#include <iostream>
#include <semaphore.h>
#include <errno.h>

typedef struct
{
	int id;
	int jobs;
	ThreadPool *tp;
	unsigned int time;
	Timer timer;
} arg_t;

static arg_t *pthread_arg;

using namespace std;

static void *jobthread(void *a)
{
	arg_t *arg = (arg_t*)a;
	int id = arg->id;
	ThreadPool *tp = arg->tp;

	while(tp->run)
	{
		// cout << "Thread " << id << " waiting for jobs " << endl;
		if(sem_trywait(&tp->availablejobs) < 0)
		{
			// Mark thread as being idle
			sem_post(&tp->idlethreads);
			sem_wait(&tp->availablejobs);
		}
		
		// Mark thread as running
		sem_post(&tp->runningthreads);
		

		int rthreads;

		sem_getvalue(&tp->runningthreads, &rthreads);

		// cout << "Running threads " << rthreads << endl;

		if(tp->run)
		{
			pthread_mutex_lock(&tp->mutex);
				Job *job = tp->jobs.front();
				tp->jobs.pop_front();
			pthread_mutex_unlock(&tp->mutex);

			arg->timer.Mark();

			job->Run(id);

			arg->time += arg->timer.GetElapsed();

			if(job->Free()) delete job;

			arg->jobs++;
		}
		
		sem_wait(&tp->runningthreads);

	}

	pthread_exit(NULL);
}


ThreadPool::ThreadPool(int numthreads)
{
	Init(numthreads);
}

void ThreadPool::Init(int numthreads)
{
	pthread_attr_t attr;

	run = true;

	this->numthreads = numthreads;

	pthread_mutex_init(&mutex, NULL);	

	sem_init(&availablejobs, 0, 0);
	sem_init(&idlethreads, 0, 0);
	sem_init(&runningthreads, 0, 0);

	pthreads = new pthread_t[numthreads];
	pthread_arg = new arg_t[numthreads];

	/* Initialize and set thread detached attribute */
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

	for(int i=0; i < numthreads; i++)
	{
		pthread_arg[i].id = i;
		pthread_arg[i].tp = this;
		pthread_arg[i].jobs = 0;
		pthread_arg[i].time = 0;

		pthread_create(&pthreads[i], &attr, jobthread, &pthread_arg[i]);
	}

	pthread_attr_destroy(&attr);	
}

void ThreadPool::SetNumThreads(int numthreads)
{
	if(numthreads != this->numthreads)
	{
		Exit();
		Init(numthreads);
	}
}

void ThreadPool::Add(Job *job)
{
	pthread_mutex_lock(&mutex);
		jobs.push_back(job);		
	pthread_mutex_unlock(&mutex);	
	
	sem_post(&availablejobs);
}

void ThreadPool::Wait()
{
	int rthreads;
	int ajobs;
	int ithreads;	

	do
	{

		bool run = true;

		// timer.Mark();

		do
		{
			sem_getvalue(&runningthreads, &rthreads);
			sem_getvalue(&availablejobs, &ajobs);

			// cout << "Running threads: " << rthreads << " Available jobs: " << ajobs << endl;

			if(rthreads > 0 || ajobs > 0)
				sem_wait(&idlethreads);
			else
				run = false;

		}
		while(run);

		sem_getvalue(&idlethreads, &ithreads);

		for(int i=0; i < ithreads; i++) sem_wait(&idlethreads);

		sem_getvalue(&runningthreads, &rthreads);
		sem_getvalue(&idlethreads, &ithreads);
		sem_getvalue(&availablejobs, &ajobs);

	} while ( rthreads > 0 || ithreads > 0 || ajobs > 0);

	// cout << "Running threads: " << rthreads << " Idle threads: " << ithreads << " Available jobs: " << ajobs << endl;

	// cout << "Stats" << endl;
	// cout << "Wait: " << timer.GetElapsed() << endl;

	for(int i=0; i < numthreads; i++)
	{
		// cout << "Thread " << pthread_arg[i].id << " executed " << pthread_arg[i].jobs << " jobs " << " time " << pthread_arg[i].time << endl;
		pthread_arg[i].jobs = 0;
		pthread_arg[i].time = 0;
	}


}

void ThreadPool::Exit()
{
	int ajobs;

	sem_getvalue(&availablejobs, &ajobs);

	if(ajobs > 0) Wait();

	run = false;

	for(int i=0; i < numthreads; i++)
	{
		sem_post(&availablejobs);		
	}

	for(int i=0; i < numthreads; i++)
	{
		// cout << "Waiting for thread " << i << " to finish" << endl;

		pthread_join(pthreads[i], NULL);
	}

	sem_destroy(&availablejobs);
	sem_destroy(&idlethreads);
	sem_destroy(&runningthreads);

	delete [] pthreads;
	delete [] pthread_arg;
}


