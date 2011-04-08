#ifndef _LZMATH_LZMATRIX_H_
#define _LZMATH_LZMATRIX_H_

namespace lzmath
{
	class Matrix4x4f;
}

class lzmath::Matrix4x4f
{
	public:
		float m[4][4] __attribute__((aligned(16))) ;
	public:
		Matrix4x4f();
		Matrix4x4f(float mtx[4][4]);
		Matrix4x4f(	float m00, float m01, float m02, float m03,
					float m10, float m11, float m12, float m13,
					float m20, float m21, float m22, float m23,
					float m30, float m31, float m32, float m33);					

		Matrix4x4f* Transpose() const;
		Matrix4x4f* Inverse() const;

		Matrix4x4f operator*(const Matrix4x4f &mtx) const;

		void Print();

};

#endif

