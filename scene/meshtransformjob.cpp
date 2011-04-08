#include <scene/meshtransformjob.h>
#include <scene/scene.h>

void MeshTransformJob::Init(int tid, int nthreads)
{
	this->nthreads = nthreads;
	this->tid = tid;

}

void MeshTransformJob::Run(int thread_id)
{
	#ifdef _CELL
	if(GetScene()->meshtransform_kernel >= K_SPU_MESHTRANSFORM)
		MeshTransformSPU();
	else
	#endif
		MeshTransform();
}

bool MeshTransformJob::Free()
{
	return false;
}


