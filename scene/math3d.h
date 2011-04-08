#ifndef _LZ_MATH3D_H_
#define _LZ_MATH3D_H_

#include <share/structs.h>
#include <math.h>

#ifdef _CELL
	#include <altivec.h>
	#include <spu2vmx.h>
	#include <normalize3.h>
	#include <xform_vec3.h>
	#include <xform_vec4.h>
	#include <transpose_matrix4x4.h>
#endif

#ifdef _CELL

inline vector float spu_max(vector float a, vector float b)
{
	return spu_sel( b, a, spu_cmpgt( a, b ) );
}

inline vector float spu_min(vector float a, vector float b)
{
	return spu_sel( a, b, spu_cmpgt( a, b ) );
}
#endif


inline void MultMatrixVector(float matrix[4][4], float v[3])
{
	float ret[4] ALIGNED(16);
	
	ret[0] = matrix[0][0]*v[0] + matrix[0][1]*v[1] + matrix[0][2]*v[2];
	ret[1] = matrix[1][0]*v[0] + matrix[1][1]*v[1] + matrix[1][2]*v[2];
	ret[2] = matrix[2][0]*v[0] + matrix[2][1]*v[1] + matrix[2][2]*v[2];

	v[0] = ret[0];
	v[1] = ret[1];
	v[2] = ret[2];
}

#ifdef _CELL
inline void MultMatrixVectorSIMD(float matrix[4][4], float v[3])
{
	float ret[4] ALIGNED(16);
	float va[4] ALIGNED(16);

	va[0] = v[0];
	va[1] = v[1];
	va[2] = v[2];
	va[3] = 0;

	vector float *m =  (vector float *)matrix;
	vector float *vv = (vector float *)va;
	vector float *rv = (vector float *)ret;

	vector float tm[4];
	_transpose_matrix4x4(tm, m);

	*rv = _xform_vec4(*vv, tm);
	
	v[0] = ret[0];
	v[1] = ret[1];
	v[2] = ret[2];
}
#endif


inline void MultMatrixPoint(float matrix[4][4], float v[4])
{
	float ret[4] ALIGNED(16);
	
	ret[0] = matrix[0][0]*v[0] + matrix[0][1]*v[1] + matrix[0][2]*v[2] + matrix[0][3];
	ret[1] = matrix[1][0]*v[0] + matrix[1][1]*v[1] + matrix[1][2]*v[2] + matrix[1][3];
	ret[2] = matrix[2][0]*v[0] + matrix[2][1]*v[1] + matrix[2][2]*v[2] + matrix[2][3];
	
	v[0] = ret[0];
	v[1] = ret[1];
	v[2] = ret[2];
}

inline void MultMatrixPoint(float matrix[4][4], float v[4], float ret[4])
{
	ret[0] = matrix[0][0]*v[0] + matrix[0][1]*v[1] + matrix[0][2]*v[2] + matrix[0][3];
	ret[1] = matrix[1][0]*v[0] + matrix[1][1]*v[1] + matrix[1][2]*v[2] + matrix[1][3];
	ret[2] = matrix[2][0]*v[0] + matrix[2][1]*v[1] + matrix[2][2]*v[2] + matrix[2][3];
}


#ifdef _CELL
inline void MultMatrixPointSIMD(float matrix[4][4], float v[4])
{
	float ret[4] ALIGNED(16);
	
	float va[4] ALIGNED(16);

	va[0] = v[0];
	va[1] = v[1];
	va[2] = v[2];
	va[3] = 1;

	vector float *m =  (vector float *)matrix;
	vector float *vv = (vector float *)va;
	vector float *rv = (vector float *)ret;

	vector float tm[4];
	_transpose_matrix4x4(tm, m);

	*rv = _xform_vec4(*vv, tm);

	v[0] = ret[0];
	v[1] = ret[1];
	v[2] = ret[2];
}

inline void MultMatrixPointSIMD(float matrix[4][4], float v[4], float ret[4])
{
	float va[4] ALIGNED(16);

	va[0] = v[0];
	va[1] = v[1];
	va[2] = v[2];
	va[3] = 1;

	vector float *m =  (vector float *)matrix;
	vector float *vv = (vector float *)va;
	vector float *rv = (vector float *)ret;

	vector float tm[4];
	_transpose_matrix4x4(tm, m);

	*rv = _xform_vec4(*vv, tm);

	ret[3] = 1;

}

#endif

inline void Normalize(float v[4])
{
	float len = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
	float d = 1 / len;	

	v[0] *= d;
	v[1] *= d;
	v[2] *= d;
}

#ifdef _CELL
inline void NormalizeSIMD(float v[4])
{
	vector float *vv = (vector float*)v;
	*vv = _normalize3(*vv);
}
#endif


inline float VectorLen(float *v)
{
	return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

inline void VectorNormalize(float *v)
{
	float len = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
	float invlen = 1/len;

	v[0] *= invlen;
	v[1] *= invlen;
	v[2] *= invlen;
}


inline void VectorSub(float *v1, float *v2, float *ret)
{
	ret[0] = v1[0] - v2[0]; 
	ret[1] = v1[1] - v2[1]; 
	ret[2] = v1[2] - v2[2]; 
}

inline void VectorScale(float *v, float s, float *ret)
{
	ret[0] = v[0]*s;
	ret[1] = v[1]*s;
	ret[2] = v[2]*s;
}


inline void VectorCross(float *v1, float *v2, float *ret)
{
	ret[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
	ret[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
	ret[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

inline float VectorDot(float *v1, float *v2)
{
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] *v2[2];
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

inline void MatrixTranspose(float m[4][4])
{
	float ret[4][4];

	MatrixTranspose(m, ret);

	for(int i=0; i < 4; i++)
		for(int j=0; j < 4; j++)
			m[i][j] = ret[i][j];
}




#endif


