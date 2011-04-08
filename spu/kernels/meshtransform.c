#include <stdio.h>
#include <meshtransform.h>
#include <spu_intrinsics.h>
#include <spu_mfcio.h>
#include <xform_vec3.h>
#include <xform_vec4.h>
#include <transpose_matrix4x4.h>
#include <dma.h>

#define BUFFER_SIZE	100

typedef unsigned int uint;

volatile meshtransform_arg_t arg;

triangle_t in_tbuffer[2][BUFFER_SIZE];
triangle_t out_tbuffer[2][BUFFER_SIZE];
aabb_t abuffer[2][BUFFER_SIZE];
normal_t in_nbuffer[2][BUFFER_SIZE];
normal_t out_nbuffer[2][BUFFER_SIZE];


struct mfc_list_element in_tlist[2][16] ALIGNED(16);
struct mfc_list_element out_tlist[2][16] ALIGNED(16);
struct mfc_list_element alist[2][16] ALIGNED(16);
struct mfc_list_element in_nlist[2][16] ALIGNED(16);
struct mfc_list_element out_nlist[2][16] ALIGNED(16);


inline vector float spu_max(vector float a, vector float b)
{
	return spu_sel( b, a, spu_cmpgt( a, b ) );
}

inline vector float spu_min(vector float a, vector float b)
{
	return spu_sel( a, b, spu_cmpgt( a, b ) );
}

inline void MultMatrixPointSIMD(vector float matrix[4], float v[4], float ret[4])
{
	vector float *vv = (vector float *)v;
	vector float *vret = (vector float *)ret;	

	*vret = _xform_vec4(*vv, matrix);
}

inline void MultMatrixPoint(float matrix[4][4], float v[4], float ret[4])
{
	ret[0] = matrix[0][0]*v[0] + matrix[0][1]*v[1] + matrix[0][2]*v[2] + matrix[0][3];
	ret[1] = matrix[1][0]*v[0] + matrix[1][1]*v[1] + matrix[1][2]*v[2] + matrix[1][3];
	ret[2] = matrix[2][0]*v[0] + matrix[2][1]*v[1] + matrix[2][2]*v[2] + matrix[2][3];
}

inline void MatrixTranspose(float m[4][4], float ret[4][4])
{
	ret[0][0] = m[0][0];
	ret[0][1] = m[1][0];
	ret[0][2] = m[2][0];
	ret[0][3] = m[3][0];

	ret[1][0] = m[0][1];
	ret[1][1] = m[1][1];
	ret[1][2] = m[2][1];
	ret[1][3] = m[3][1];

	ret[2][0] = m[0][2];
	ret[2][1] = m[1][2];
	ret[2][2] = m[2][2];
	ret[2][3] = m[3][2];

	ret[3][0] = m[0][3];
	ret[3][1] = m[1][3];
	ret[3][2] = m[2][3];
	ret[3][3] = m[3][3];
}

inline float max(float a, float b)
{
	if(a > b)
		return a;
	else
		return b;
}

inline float min(float a, float b)
{
	if(a < b)
		return a;
	else
		return b;
}



void Transform(vector float matrix[4], vector float invmatrix[4], triangle_t *t_src, triangle_t *t_dest, aabb_t *aabb, normal_t *n_src, normal_t *n_dest, int numtriangles)
{
	int i = 0;

	if(arg.simd == 1)
	{
		for(i=0; i < numtriangles; i++)
		{
			MultMatrixPointSIMD(matrix, t_src[i].v1, t_dest[i].v1);
			MultMatrixPointSIMD(matrix, t_src[i].v2, t_dest[i].v2);
			MultMatrixPointSIMD(matrix, t_src[i].v3, t_dest[i].v3);

			MultMatrixPointSIMD(invmatrix, n_src[i].n1, n_dest[i].n1);
			MultMatrixPointSIMD(invmatrix, n_src[i].n2, n_dest[i].n2);
			MultMatrixPointSIMD(invmatrix, n_src[i].n3, n_dest[i].n3);

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
	else
	{
		float fmatrix[4][4];
		float finvmatrix[4][4];

		vector float *ptr = (vector float*)fmatrix;

		ptr[0] = arg.matrix[0];
		ptr[1] = arg.matrix[1];
		ptr[2] = arg.matrix[2];
		ptr[3] = arg.matrix[3];

		ptr = (vector float*)finvmatrix;
	
		ptr[0] = arg.invmatrix[0];
		ptr[1] = arg.invmatrix[1];
		ptr[2] = arg.invmatrix[2];
		ptr[3] = arg.invmatrix[3];

		float tinvmatrix[4][4] ALIGNED(16);

		MatrixTranspose(finvmatrix, tinvmatrix);

		for(i=0; i < numtriangles; i++)
		{
			MultMatrixPoint(fmatrix, t_src[i].v1, t_dest[i].v1);
			MultMatrixPoint(fmatrix, t_src[i].v2, t_dest[i].v2);
			MultMatrixPoint(fmatrix, t_src[i].v3, t_dest[i].v3);

			MultMatrixPoint(finvmatrix, n_src[i].n1, n_dest[i].n1);
			MultMatrixPoint(finvmatrix, n_src[i].n2, n_dest[i].n2);
			MultMatrixPoint(finvmatrix, n_src[i].n3, n_dest[i].n3);

			aabb[i].min[0] = t_dest[i].v1[0];
			aabb[i].min[1] = t_dest[i].v1[1];
			aabb[i].min[2] = t_dest[i].v1[2];

			aabb[i].max[0] = t_dest[i].v1[0];
			aabb[i].max[1] = t_dest[i].v1[1];
			aabb[i].max[2] = t_dest[i].v1[2];

			aabb[i].min[0] = min(aabb[i].min[0], t_dest[i].v2[0]);
			aabb[i].min[1] = min(aabb[i].min[1], t_dest[i].v2[1]);
			aabb[i].min[2] = min(aabb[i].min[2], t_dest[i].v2[2]);

			aabb[i].max[0] = max(aabb[i].max[0], t_dest[i].v2[0]);
			aabb[i].max[1] = max(aabb[i].max[1], t_dest[i].v2[1]);
			aabb[i].max[2] = max(aabb[i].max[2], t_dest[i].v2[2]);

			aabb[i].min[0] = min(aabb[i].min[0], t_dest[i].v3[0]);
			aabb[i].min[1] = min(aabb[i].min[1], t_dest[i].v3[1]);
			aabb[i].min[2] = min(aabb[i].min[2], t_dest[i].v3[2]);

			aabb[i].max[0] = max(aabb[i].max[0], t_dest[i].v3[0]);
			aabb[i].max[1] = max(aabb[i].max[1], t_dest[i].v3[1]);
			aabb[i].max[2] = max(aabb[i].max[2], t_dest[i].v3[2]);
		}



	}

}


int main(unsigned long long spu_id __attribute__ ((unused)), unsigned long long parm)
{
	uint in_t_tag[2];
	uint out_t_tag[2];
	uint a_tag[2];
	uint in_n_tag[2];
	uint out_n_tag[2];

	in_t_tag[0] = mfc_tag_reserve();
	in_t_tag[1] = mfc_tag_reserve();

	out_t_tag[0] = mfc_tag_reserve();
	out_t_tag[1] = mfc_tag_reserve();

	a_tag[0] = mfc_tag_reserve();
	a_tag[1] = mfc_tag_reserve();

	in_n_tag[0] = mfc_tag_reserve();
	in_n_tag[1] = mfc_tag_reserve();

	out_n_tag[0] = mfc_tag_reserve();
	out_n_tag[1] = mfc_tag_reserve();


	uint tag_id = mfc_tag_reserve();

	// Transfer arg
	spu_mfcdma32(&arg, parm, sizeof(meshtransform_arg_t), tag_id, MFC_GET_CMD);
	DmaWait(tag_id);

	//printf("SPE MeshTransform - triangles %i\n", arg.numtriangles);

	vector float tmatrix[4];
	vector float tinvmatrix[4];

	_transpose_matrix4x4(tmatrix, arg.matrix);
	_transpose_matrix4x4(tinvmatrix, arg.invmatrix);
	

	int numtriangles = arg.numtriangles;
	int offset = 0;
	int tsize[2];	
	int toffset[2];
	
	// Get Buffer 1
	tsize[0] = (numtriangles < BUFFER_SIZE) ? numtriangles : BUFFER_SIZE;
	DmaGet(in_tlist[0], in_tbuffer[0], (uint)&arg.t_src[offset], tsize[0] * sizeof(triangle_t), in_t_tag[0]);
	DmaGet(in_nlist[0], in_nbuffer[0], (uint)&arg.n_src[offset], tsize[0] * sizeof(normal_t), in_n_tag[0]);

	toffset[0] = offset;	
	offset += tsize[0];
	numtriangles -= tsize[0];

	// Get Buffer 2
	tsize[1] = (numtriangles < BUFFER_SIZE) ? numtriangles : BUFFER_SIZE;
	DmaGet(in_tlist[1], in_tbuffer[1], (uint)&arg.t_src[offset], tsize[1] * sizeof(triangle_t), in_t_tag[1]);
	DmaGet(in_nlist[1], in_nbuffer[1], (uint)&arg.n_src[offset], tsize[1] * sizeof(normal_t), in_n_tag[1]);

	toffset[1] = offset;	
	offset += tsize[1];
	numtriangles -= tsize[1];

	while(numtriangles > 0)
	{
		// Process Buffer 1
		DmaWait(in_t_tag[0]);
		DmaWait(a_tag[0]);
		DmaWait(in_n_tag[0]);

		Transform(tmatrix, tinvmatrix, in_tbuffer[0], out_tbuffer[0], abuffer[0], in_nbuffer[0], out_nbuffer[0], tsize[0]);	

		DmaWait(out_t_tag[0]);
		DmaWait(a_tag[0]);
		DmaWait(out_n_tag[0]);

		// Put Buffer 1
		DmaPut(out_tlist[0], out_tbuffer[0], (uint)&arg.t_dest[toffset[0]], tsize[0] * sizeof(triangle_t), out_t_tag[0]);
		DmaPut(alist[0], abuffer[0], (uint)&arg.aabb[toffset[0]], tsize[0] * sizeof(aabb_t), a_tag[0]);
		DmaPut(out_nlist[0], out_nbuffer[0], (uint)&arg.n_dest[toffset[0]], tsize[0] * sizeof(normal_t), out_n_tag[0]);

		// Get Buffer 1
		tsize[0] = (numtriangles < BUFFER_SIZE) ? numtriangles : BUFFER_SIZE;
		DmaGet(in_tlist[0], in_tbuffer[0], (uint)&arg.t_src[offset], tsize[0] * sizeof(triangle_t), in_t_tag[0]);
		DmaGet(in_nlist[0], in_nbuffer[0], (uint)&arg.n_src[offset], tsize[0] * sizeof(normal_t), in_n_tag[0]);

		toffset[0] = offset;	
		offset += tsize[0];
		numtriangles -= tsize[0];

		// Process Buffer 2
		DmaWait(in_t_tag[1]);
		DmaWait(a_tag[1]);
		DmaWait(in_n_tag[1]);

		Transform(tmatrix, tinvmatrix, in_tbuffer[1], out_tbuffer[1], abuffer[1], in_nbuffer[1], out_nbuffer[1], tsize[1]);	

		DmaWait(out_t_tag[1]);
		DmaWait(a_tag[1]);
		DmaWait(out_n_tag[1]);

		// Put Buffer 1
		DmaPut(out_tlist[1], out_tbuffer[1], (uint)&arg.t_dest[toffset[1]], tsize[1] * sizeof(triangle_t), out_t_tag[1]);
		DmaPut(alist[1], abuffer[1], (uint)&arg.aabb[toffset[1]], tsize[1] * sizeof(aabb_t), a_tag[1]);
		DmaPut(out_nlist[1], out_nbuffer[1], (uint)&arg.n_dest[toffset[1]], tsize[1] * sizeof(normal_t), out_n_tag[1]);

		// Get Buffer 1
		tsize[1] = (numtriangles < BUFFER_SIZE) ? numtriangles : BUFFER_SIZE;
		DmaGet(in_tlist[1], in_tbuffer[1], (uint)&arg.t_src[offset], tsize[1] * sizeof(triangle_t), in_t_tag[1]);
		DmaGet(in_nlist[1], in_nbuffer[1], (uint)&arg.n_src[offset], tsize[1] * sizeof(normal_t), in_n_tag[1]);

		toffset[1] = offset;	
		offset += tsize[1];
		numtriangles -= tsize[1];
	}

	// Process Buffer 1
	DmaWait(in_t_tag[0]);
	DmaWait(a_tag[0]);
	DmaWait(in_n_tag[0]);

	Transform(tmatrix, tinvmatrix, in_tbuffer[0], out_tbuffer[0], abuffer[0], in_nbuffer[0], out_nbuffer[0], tsize[0]);	

	// Put Buffer 1
	DmaPut(out_tlist[0], out_tbuffer[0], (uint)&arg.t_dest[toffset[0]], tsize[0] * sizeof(triangle_t), out_t_tag[0]);
	DmaPut(alist[0], abuffer[0], (uint)&arg.aabb[toffset[0]], tsize[0] * sizeof(aabb_t), a_tag[0]);
	DmaPut(out_nlist[0], out_nbuffer[0], (uint)&arg.n_dest[toffset[0]], tsize[0] * sizeof(normal_t), out_n_tag[0]);

	// Process Buffer 2
	DmaWait(in_t_tag[1]);
	DmaWait(a_tag[1]);
	DmaWait(in_n_tag[1]);

	Transform(tmatrix, tinvmatrix, in_tbuffer[1], out_tbuffer[1], abuffer[1], in_nbuffer[1], out_nbuffer[1], tsize[1]);	

	// Put Buffer 1
	DmaPut(out_tlist[1], out_tbuffer[1], (uint)&arg.t_dest[toffset[1]], tsize[1] * sizeof(triangle_t), out_t_tag[1]);
	DmaPut(alist[1], abuffer[1], (uint)&arg.aabb[toffset[1]], tsize[1] * sizeof(aabb_t), a_tag[1]);
	DmaPut(out_nlist[1], out_nbuffer[1], (uint)&arg.n_dest[toffset[1]], tsize[1] * sizeof(normal_t), out_n_tag[1]);

	DmaWaitAll();


	return 0;
}

