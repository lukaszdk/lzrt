#include <scene/math3d.h>
#include <scene/kdbuildjob.h>
#include <scene/scene.h>
#include <share/structs.h>
#ifdef _CELL
	#include <spu/kernels/meshtransform.h>
#endif

#ifdef _CELL
extern spe_program_handle_t meshtransform;
#endif


#ifdef _CELL
void DoTransformSPU(float matrix[4][4], float invmatrix[4][4], triangle_t *t_src, triangle_t *t_dest, aabb_t *aabb, normal_t *n_src, normal_t *n_dest, int numtriangles, int tid)
{
	float tinvmatrix[4][4] ALIGNED(16);
	float m[4][4] ALIGNED(16);

	memcpy(m, matrix, sizeof(float[4][4]));
	MatrixTranspose(invmatrix, tinvmatrix);

	vector float *vm = (vector float*)m;
	vector float *vim = (vector float*)tinvmatrix;

	meshtransform_arg_t arg ALIGNED(16);

	arg.matrix[0] = vm[0];
	arg.matrix[1] = vm[1];
	arg.matrix[2] = vm[2];
	arg.matrix[3] = vm[3];

	arg.invmatrix[0] = vim[0];
	arg.invmatrix[1] = vim[1];
	arg.invmatrix[2] = vim[2];
	arg.invmatrix[3] = vim[3];

	arg.t_src = t_src;
	arg.t_dest = t_dest;
	arg.aabb = aabb;
	arg.n_src = n_src;
	arg.n_dest = n_dest;
	arg.numtriangles = numtriangles;

	if(GetScene()->meshtransform_kernel == K_MESHTRANSFORM_SIMD_SPU)
		arg.simd = 1;
	else
		arg.simd = 0;

	// Load and Run SPE program
	SPEProgram spe_meshtransform(&meshtransform);
	spe_meshtransform.Run(GetSPEContext(tid), &arg);	
}

void DoTransformSIMD(float matrix[4][4], float invmatrix[4][4], triangle_t *t_src, triangle_t *t_dest, aabb_t *aabb, normal_t *n_src, normal_t *n_dest, int numtriangles)
{
	float tinvmatrix[4][4] ALIGNED(16);
	float m[4][4] ALIGNED(16);

	memcpy(m, matrix, sizeof(float[4][4]));

	MatrixTranspose(invmatrix, tinvmatrix);

	for(int i=0; i < numtriangles; i++)
	{
		MultMatrixPointSIMD(m, t_src[i].v1, t_dest[i].v1);
		MultMatrixPointSIMD(m, t_src[i].v2, t_dest[i].v2);
		MultMatrixPointSIMD(m, t_src[i].v3, t_dest[i].v3);

		MultMatrixPointSIMD(tinvmatrix, n_src[i].n1, n_dest[i].n1);
		MultMatrixPointSIMD(tinvmatrix, n_src[i].n2, n_dest[i].n2);
		MultMatrixPointSIMD(tinvmatrix, n_src[i].n3, n_dest[i].n3);

		vector float *aabb_min = (vector float*)aabb[i].min;
		vector float *aabb_max = (vector float*)aabb[i].max;
		vector float v1 = *((vector float*)t_dest[i].v1);
		vector float v2 = *((vector float*)t_dest[i].v2);
		vector float v3 = *((vector float*)t_dest[i].v3);

		*aabb_min = v1;
		*aabb_max = v1;

		*aabb_min = spu_min(*aabb_min, v2);
		*aabb_max = spu_max(*aabb_max, v2);

		*aabb_min = spu_min(*aabb_min, v3);
		*aabb_max = spu_max(*aabb_max, v3);
	}
}
#endif


void DoTransform(float matrix[4][4], float invmatrix[4][4], triangle_t *t_src, triangle_t *t_dest, aabb_t *aabb, normal_t *n_src, normal_t *n_dest, int numtriangles)
{
	float tinvmatrix[4][4] ALIGNED(16);

	MatrixTranspose(invmatrix, tinvmatrix);

	for(int i=0; i < numtriangles; i++)
	{
		MultMatrixPoint(matrix, t_src[i].v1, t_dest[i].v1);
		MultMatrixPoint(matrix, t_src[i].v2, t_dest[i].v2);
		MultMatrixPoint(matrix, t_src[i].v3, t_dest[i].v3);

		MultMatrixPoint(tinvmatrix, n_src[i].n1, n_dest[i].n1);
		MultMatrixPoint(tinvmatrix, n_src[i].n2, n_dest[i].n2);
		MultMatrixPoint(tinvmatrix, n_src[i].n3, n_dest[i].n3);

		aabb[i].min[0] = t_dest[i].v1[0];
		aabb[i].min[1] = t_dest[i].v1[1];
		aabb[i].min[2] = t_dest[i].v1[2];

		aabb[i].max[0] = t_dest[i].v1[0];
		aabb[i].max[1] = t_dest[i].v1[1];
		aabb[i].max[2] = t_dest[i].v1[2];

		aabb[i].min[0] = fmin(aabb[i].min[0], t_dest[i].v2[0]);
		aabb[i].min[1] = fmin(aabb[i].min[1], t_dest[i].v2[1]);
		aabb[i].min[2] = fmin(aabb[i].min[2], t_dest[i].v2[2]);

		aabb[i].max[0] = fmax(aabb[i].max[0], t_dest[i].v2[0]);
		aabb[i].max[1] = fmax(aabb[i].max[1], t_dest[i].v2[1]);
		aabb[i].max[2] = fmax(aabb[i].max[2], t_dest[i].v2[2]);

		aabb[i].min[0] = fmin(aabb[i].min[0], t_dest[i].v3[0]);
		aabb[i].min[1] = fmin(aabb[i].min[1], t_dest[i].v3[1]);
		aabb[i].min[2] = fmin(aabb[i].min[2], t_dest[i].v3[2]);

		aabb[i].max[0] = fmax(aabb[i].max[0], t_dest[i].v3[0]);
		aabb[i].max[1] = fmax(aabb[i].max[1], t_dest[i].v3[1]);
		aabb[i].max[2] = fmax(aabb[i].max[2], t_dest[i].v3[2]);
	}
}

#ifdef _CELL
void MeshTransformJob::MeshTransformSPU()
{
	vector<Mesh*>::iterator it;

	for(it = GetScene()->meshlist.begin(); it != GetScene()->meshlist.end(); it++)
	{
		Mesh *m = *it;
		
		triangle_t *t_src = m->GetOrgTriangles();
		triangle_t *t_dest = m->GetTriangles();
		aabb_t *aabb = m->GetAABBs();
		normal_t *n_src = m->GetOrgNormals();
		normal_t *n_dest = m->GetNormals();
		
		int wsize = m->NumTriangles()/nthreads;
		int idx = tid * wsize;

		if(tid == nthreads-1)
		{
			wsize = m->NumTriangles() - idx;
		}	
		
		DoTransformSPU(m->transform.matrix->m, m->transform.invMatrix->m, &t_src[idx], &t_dest[idx], &aabb[idx], &n_src[idx], &n_dest[idx], wsize, tid);
	}
}
#endif


void MeshTransformJob::MeshTransform()
{
	vector<Mesh*>::iterator it;

	for(it = GetScene()->meshlist.begin(); it != GetScene()->meshlist.end(); it++)
	{
		Mesh *m = *it;
		
		triangle_t *t_src = m->GetOrgTriangles();
		triangle_t *t_dest = m->GetTriangles();
		aabb_t *aabb = m->GetAABBs();
		normal_t *n_src = m->GetOrgNormals();
		normal_t *n_dest = m->GetNormals();
		
		// cout << "numtriangles " << m->NumTriangles() << endl;

		int wsize = m->NumTriangles()/nthreads;
		int idx = tid * wsize;

		if(tid == nthreads-1)
		{
			wsize = m->NumTriangles() - idx;
		}	
		
		// cout << "thread " << tid << " idx " << idx << " wsize " << wsize << endl;

		#ifdef _CELL
		if(GetScene()->meshtransform_kernel == K_MESHTRANSFORM_SIMD)
			DoTransformSIMD(m->transform.matrix->m, m->transform.invMatrix->m, &t_src[idx], &t_dest[idx], &aabb[idx], &n_src[idx], &n_dest[idx], wsize);
		else
		#endif
			DoTransform(m->transform.matrix->m, m->transform.invMatrix->m, &t_src[idx], &t_dest[idx], &aabb[idx], &n_src[idx], &n_dest[idx], wsize);
	}
}


