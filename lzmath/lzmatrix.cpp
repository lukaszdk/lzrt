#include <lzmatrix.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <iostream>

using namespace lzmath;

using namespace std;

Matrix4x4f::Matrix4x4f()
{
	for(int i=0; i < 4; i++)
		for(int j=0; j < j; j++)
		{
			if(i == j)
				m[i][j] = 1;
			else
				m[i][j] = 0;
		}
}

Matrix4x4f::Matrix4x4f(float mtx[4][4])
{
	for(int i=0; i < 4; i++)
		for(int j=0; j < 4; j++)
			m[i][j] = mtx[i][j];
}

Matrix4x4f::Matrix4x4f(	float m00, float m01, float m02, float m03,
					float m10, float m11, float m12, float m13,
					float m20, float m21, float m22, float m23,
					float m30, float m31, float m32, float m33)
{
	m[0][0] = m00;
	m[0][1] = m01;
	m[0][2] = m02;
	m[0][3] = m03;

	m[1][0] = m10;
	m[1][1] = m11;
	m[1][2] = m12;
	m[1][3] = m13;

	m[2][0] = m20;
	m[2][1] = m21;
	m[2][2] = m22;
	m[2][3] = m23;

	m[3][0] = m30;
	m[3][1] = m31;
	m[3][2] = m32;
	m[3][3] = m33;
}


Matrix4x4f* Matrix4x4f::Transpose() const
{
	return new Matrix4x4f(		m[0][0], m[1][0], m[2][0], m[3][0],
								m[0][1], m[1][1], m[2][1], m[3][1],
								m[0][2], m[1][2], m[2][2], m[3][2],
								m[0][3], m[1][3], m[2][3], m[3][3]);
}

Matrix4x4f* Matrix4x4f::Inverse() const
{
	int indxc[4], indxr[4];
	int ipiv[4] = { 0, 0, 0, 0 };
	float minv[4][4];
	memcpy(minv, m, 4*4*sizeof(float));
	for (int i = 0; i < 4; i++) {
		int irow = -1, icol = -1;
		float big = 0.;
		// Choose pivot
		for (int j = 0; j < 4; j++) {
			if (ipiv[j] != 1) {
				for (int k = 0; k < 4; k++) {
					if (ipiv[k] == 0) {
						if (fabsf(minv[j][k]) >= big) {
							big = float(fabsf(minv[j][k]));
							irow = j;
							icol = k;
						}
					}
					else if (ipiv[k] > 1)
						fprintf(stderr, "Singular matrix in Matrix4x4f::Inverse");
				}
			}
		}
		++ipiv[icol];
		// Swap rows _irow_ and _icol_ for pivot
		if (irow != icol) {
			for (int k = 0; k < 4; ++k)
			{
				float tmp = minv[irow][k];
				minv[irow][k] = minv[icol][k];
				minv[icol][k] = tmp;

			}
		}
		indxr[i] = irow;
		indxc[i] = icol;
		if (minv[icol][icol] == 0.)
			fprintf(stderr, "Singular matrix in Matrix4x4f::Inverse");
		// Set $m[icol][icol]$ to one by scaling row _icol_ appropriately
		float pivinv = 1.f / minv[icol][icol];
		minv[icol][icol] = 1.f;
		for (int j = 0; j < 4; j++)
			minv[icol][j] *= pivinv;
		// Subtract this row from others to zero out their columns
		for (int j = 0; j < 4; j++) {
			if (j != icol) {
				float save = minv[j][icol];
				minv[j][icol] = 0;
				for (int k = 0; k < 4; k++)
					minv[j][k] -= minv[icol][k]*save;
			}
		}
	}
	// Swap columns to reflect permutation
	for (int j = 3; j >= 0; j--) {
		if (indxr[j] != indxc[j]) {
			for (int k = 0; k < 4; k++)
			{
				float tmp = minv[k][indxr[j]];
				minv[k][indxr[j]] = minv[k][indxc[j]];
				minv[k][indxc[j]] = tmp;
			}
		}
	}

	return new Matrix4x4f(minv);
}



Matrix4x4f Matrix4x4f::operator*(const Matrix4x4f &mtx) const
{
	float r[4][4];

	for(int i=0; i < 4; i++)
		for(int j=0; j < 4; j++)
		{
			r[i][j]  = m[i][0] * mtx.m[0][j];
			r[i][j] += m[i][1] * mtx.m[1][j];
			r[i][j] += m[i][2] * mtx.m[2][j];
			r[i][j] += m[i][3] * mtx.m[3][j];
		}

	return Matrix4x4f(r);
}

void Matrix4x4f::Print()
{
	for(int i=0; i < 4; i++)
	{
		cout << "[" << m[i][0] << " " << m[i][1] << " " << m[i][2] << " " << m[i][3] << "]" << endl;
	}


}


