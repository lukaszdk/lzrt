#include <scene/kdbuildjob.h>
#include <scene/scene.h>
#include <iostream>

extern int KDGetNumNodes();
extern int KDGetNumLeaves();

KDBuildJob::KDBuildJob()
{
	#ifdef _CELL
		aabb_buffer[0] = (buffer_t*)_malloc_align(sizeof(buffer_t), 4);
		aabb_buffer[1] = (buffer_t*)_malloc_align(sizeof(buffer_t), 4);
		kdbuffer[0] = (buffer_t*)_malloc_align(sizeof(buffer_t), 4);
		kdbuffer[1] = (buffer_t*) _malloc_align(sizeof(buffer_t), 4);
		leaf_aabb_buffer = (buffer_t*)_malloc_align(sizeof(buffer_t), 4);
		leafbuffer = (buffer_t*)_malloc_align(sizeof(buffer_t), 4);
		polybuffer = (buffer_t*)_malloc_align(sizeof(buffer_t), 4);
	#else
		aabb_buffer[0] = new buffer_t(); 	
		aabb_buffer[1] = new buffer_t(); 	
		kdbuffer[0] = new buffer_t();
		kdbuffer[1] = new buffer_t();
		leaf_aabb_buffer = new buffer_t();
		leafbuffer = new buffer_t();
		polybuffer = new buffer_t();
	#endif


	BufferReset(aabb_buffer[0]);
	BufferReset(aabb_buffer[1]);
	BufferReset(kdbuffer[0]);
	BufferReset(kdbuffer[1]);
	BufferReset(leaf_aabb_buffer);
	BufferReset(leafbuffer);
	BufferReset(polybuffer);

	numbins[0] = 0;
	numbins[1] = 0;

	sem_init(&sem, 0, 0);
}

KDBuildJob::~KDBuildJob()
{
	BufferFree(aabb_buffer[0]);
	BufferFree(aabb_buffer[1]);
	BufferFree(kdbuffer[0]);
	BufferFree(kdbuffer[1]);
	BufferFree(leaf_aabb_buffer);
	BufferFree(leafbuffer);
	BufferFree(polybuffer);

	sem_destroy(&sem);
	
	#ifdef _CELL
		_free_align(aabb_buffer[0]);
		_free_align(aabb_buffer[1]);
		_free_align(kdbuffer[0]);
		_free_align(kdbuffer[1]);
		_free_align(leaf_aabb_buffer);
		_free_align(leafbuffer);
		_free_align(polybuffer);
	#else
		delete aabb_buffer[0]; 	
		delete aabb_buffer[1]; 	
		delete kdbuffer[0];
		delete kdbuffer[1];
		delete leaf_aabb_buffer;
		delete leafbuffer;
		delete polybuffer;
	#endif



}


void KDBuildJob::Init(int startnode, int startleaf, int endnode, int endleaf, bool root)
{
	this->startnode = startnode;
	this->startleaf = startleaf;
	this->curnode = startnode;
	this->curleaf = startleaf;
	njobs = 0;
	curjob = 0;
	splitaxis = 0;

	// cout << "Init endnode " << endnode << " endleaf " << endleaf << endl;

	if(!root)
	{
		this->endnode = endnode;
		this->endleaf = endleaf;
		this->root = false;

		int maxnumleaves = KDGetNumLeaves();
		int numpolys = GetScene()->numpolys;

		int bsize = (numpolys * 10) + (maxnumleaves * 50);

		BufferCreate(aabb_buffer[0], sizeof(aabb_t), bsize);
		BufferCreate(aabb_buffer[1], sizeof(aabb_t), bsize);		

		BufferCreate(kdbuffer[0], sizeof(kdbuffer_t), maxnumleaves);
		BufferCreate(kdbuffer[1], sizeof(kdbuffer_t), maxnumleaves);

		BufferCreate(leafbuffer, sizeof(kdbuffer_t), maxnumleaves);
		BufferCreate(leaf_aabb_buffer, sizeof(aabb_t), bsize);

		BufferCreate(polybuffer, sizeof(kdpoly_t), numpolys*10);
	}
	else
	{
		//cout << "root " << endl;

		// Root
		this->root = true;
		GetScene()->KDInitBuild(GetScene()->kdtree_mls, GetScene()->kdtree_etd);

		this->endnode = KDGetNumNodes();
		this->endleaf = KDGetNumLeaves();

		int maxnumleaves = KDGetNumLeaves();
		int numpolys = GetScene()->numpolys;

		int bsize = (numpolys * 10) + (maxnumleaves * 50);

		BufferCreate(aabb_buffer[0], sizeof(aabb_t), bsize);
		BufferCreate(aabb_buffer[1], sizeof(aabb_t), bsize);		

		BufferCreate(kdbuffer[0], sizeof(kdbuffer_t), maxnumleaves);
		BufferCreate(kdbuffer[1], sizeof(kdbuffer_t), maxnumleaves);

		BufferCreate(leafbuffer, sizeof(kdbuffer_t), maxnumleaves);
		BufferCreate(leaf_aabb_buffer, sizeof(aabb_t), bsize);

		BufferCreate(polybuffer, sizeof(kdpoly_t), numpolys*10);

		GetScene()->KDLoadMeshes(this, kdbuffer[0], leafbuffer, GetScene()->kdtree_mls, &curnode);
	}
}

void KDBuildJob::SetTid(int tid)
{
	this->tid = tid;
}

void KDBuildJob::AddJob(KDBuildJob *j)
{
	int node_range = endnode - curnode;
	int leaf_range = endleaf - curleaf;
	
	j->Init( curnode + (node_range/2), curleaf + (leaf_range/2), this->endnode, this->endleaf);

	this->endnode = curnode + (node_range/2);
	this->endleaf = curleaf + (leaf_range/2);

	jobs[njobs] = j;
	njobs++;
}

void KDBuildJob::Run(int thread_id)
{
	// Wait for the job to be started
	if(!root) 	sem_wait(&sem);
	
	#ifdef _CELL
	if(GetScene()->kdbuild_kernel >= K_SPU_KDBUILD)
		KDBuildSPU(GetScene()->kdtree_mls, GetScene()->kdtree_etd, root);
	else
	#endif
		KDBuild(GetScene()->kdtree_mls, GetScene()->kdtree_etd, root);
}

bool KDBuildJob::Free()
{
	return false;
}


