#include <lztransform.h>
#include <math.h>
#include <lzmatrix.h>
#include <lzvector.h>
#include <lzpoint.h>
#include <lznormal.h>
#include <lzray.h>

#ifdef __SSE3__
#include <pmmintrin.h>
#endif

#include <iostream>

using namespace std;
using namespace lzmath;

Transform::Transform()
{

}

Transform::Transform(Matrix4x4f *matrix)
{
	this->matrix = matrix;
	this->invMatrix = matrix->Inverse();

}

Transform::Transform(Matrix4x4f *matrix, Matrix4x4f *invMatrix)
{
	this->matrix = matrix;
	this->invMatrix = invMatrix;
}

Transform Transform::operator*(const Transform &t) const
{
	Matrix4x4f *matrix = new Matrix4x4f(*this->matrix * *t.matrix);
	Matrix4x4f *invMatrix = new Matrix4x4f(*t.invMatrix * *this->invMatrix);

	return Transform(matrix, invMatrix);
}

Transform Transform::GetInverse() const
{
	return Transform(invMatrix, matrix);
}



Vector3f Transform::operator()(const Vector3f &v) const
{
	return Vector3f(	matrix->m[0][0]*v.x + matrix->m[0][1]*v.y + matrix->m[0][2]*v.z,
						matrix->m[1][0]*v.x + matrix->m[1][1]*v.y + matrix->m[1][2]*v.z,
						matrix->m[2][0]*v.x + matrix->m[2][1]*v.y + matrix->m[2][2]*v.z);
}

Point3f Transform::operator()(const Point3f &p) const
{
	return Point3f(		matrix->m[0][0]*p.x + matrix->m[0][1]*p.y + matrix->m[0][2]*p.z + matrix->m[0][3],
						matrix->m[1][0]*p.x + matrix->m[1][1]*p.y + matrix->m[1][2]*p.z + matrix->m[1][3],
						matrix->m[2][0]*p.x + matrix->m[2][1]*p.y + matrix->m[2][2]*p.z + matrix->m[2][3]);	
}

void Transform::operator()(float *ret, float p[3]) const
{
	ret[0] = matrix->m[0][0]*p[0] + matrix->m[0][1]*p[1] + matrix->m[0][2]*p[2] + matrix->m[0][3];
	ret[1] = matrix->m[1][0]*p[0] + matrix->m[1][1]*p[1] + matrix->m[1][2]*p[2] + matrix->m[1][3];
	ret[2] = matrix->m[2][0]*p[0] + matrix->m[2][1]*p[1] + matrix->m[2][2]*p[2] + matrix->m[2][3];	
}

		
void Transform::operator()(Point3f *ret, const Point3f &p) const
{
	#ifdef __SSE3NOT__
		float vp[4]  __attribute__((aligned(16))) = { p.x, p.y, p.z, 0 };
		
		__m128 vec;
		__m128 mat[3];
		__m128 res[3];
		
		vec = _mm_load_ps(vp);
		
		mat[0] = _mm_loadu_ps(matrix->m[0]);
		mat[1] = _mm_loadu_ps(matrix->m[1]);
		mat[2] = _mm_loadu_ps(matrix->m[2]);
		
		res[0] = _mm_mul_ps(mat[0], vec);
		res[1] = _mm_mul_ps(mat[1], vec);
		res[2] = _mm_mul_ps(mat[2], vec);

		res[0] = _mm_hadd_ps(res[0], res[0]);
		res[0] = _mm_hadd_ps(res[0], res[0]);

		res[1] = _mm_hadd_ps(res[1], res[1]);
		res[1] = _mm_hadd_ps(res[1], res[1]);
		
		res[2] = _mm_hadd_ps(res[2], res[2]);
		res[2] = _mm_hadd_ps(res[2], res[2]);

		_mm_store_ss(&ret->x, res[0]);
		_mm_store_ss(&ret->y, res[1]);
		_mm_store_ss(&ret->z, res[2]);					

	#else
		Point3f t = Point3f(p.x, p.y, p.z);

		ret->x = matrix->m[0][0]*t.x + matrix->m[0][1]*t.y + matrix->m[0][2]*t.z + matrix->m[0][3];
		ret->y = matrix->m[1][0]*t.x + matrix->m[1][1]*t.y + matrix->m[1][2]*t.z + matrix->m[1][3];
		ret->z = matrix->m[2][0]*t.x + matrix->m[2][1]*t.y + matrix->m[2][2]*t.z + matrix->m[2][3];	
	#endif

}

void Transform::ApplyNormal(float n[3], float ret[3])
{
	ret[0] = invMatrix->m[0][0]*n[0] + invMatrix->m[1][0]*n[1] + invMatrix->m[2][0]*n[2];
	ret[1] = invMatrix->m[0][1]*n[0] + invMatrix->m[1][1]*n[1] + invMatrix->m[2][1]*n[2];
	ret[2] = invMatrix->m[0][2]*n[0] + invMatrix->m[1][2]*n[1] + invMatrix->m[2][2]*n[2];

}

Normal Transform::operator()(const Normal &n) const
{
	return Normal(		invMatrix->m[0][0]*n.x + invMatrix->m[1][0]*n.y + invMatrix->m[2][0]*n.z,
						invMatrix->m[0][1]*n.x + invMatrix->m[1][1]*n.y + invMatrix->m[2][1]*n.z,
						invMatrix->m[0][2]*n.x + invMatrix->m[1][2]*n.y + invMatrix->m[2][2]*n.z);

}

Ray Transform::operator()(const Ray &r) const
{
	Ray ret;

	ret.o = (*this)(r.o);
	ret.d = (*this)(r.d);

	ret.mint = r.mint;
	ret.maxt = r.maxt;

	return ret;
}


void Transform::operator()(Ray *ret, const Ray &r) const
{
	#ifdef __SSE3NOT__
		float ray_d[4]  __attribute__((aligned(16))) = { r.d.x, r.d.y, r.d.z, 0 };
		float ray_o[4]  __attribute__((aligned(16))) = { r.o.x, r.o.y, r.o.z, 0 };

		__m128 vec;
		__m128 mat[3];
		__m128 res[3];
		
		vec = _mm_load_ps(ray_d);
		
		mat[0] = _mm_loadu_ps(matrix->m[0]);
		mat[1] = _mm_loadu_ps(matrix->m[1]);
		mat[2] = _mm_loadu_ps(matrix->m[2]);
		
		res[0] = _mm_mul_ps(mat[0], vec);
		res[1] = _mm_mul_ps(mat[1], vec);
		res[2] = _mm_mul_ps(mat[2], vec);

		res[0] = _mm_hadd_ps(res[0], res[0]);
		res[0] = _mm_hadd_ps(res[0], res[0]);

		res[1] = _mm_hadd_ps(res[1], res[1]);
		res[1] = _mm_hadd_ps(res[1], res[1]);
		
		res[2] = _mm_hadd_ps(res[2], res[2]);
		res[2] = _mm_hadd_ps(res[2], res[2]);

		_mm_store_ss(&ret->d.x, res[0]);
		_mm_store_ss(&ret->d.y, res[1]);
		_mm_store_ss(&ret->d.z, res[2]);				 	 	

		
		vec = _mm_load_ps(ray_o);
		
		res[0] = _mm_mul_ps(mat[0], vec);
		res[1] = _mm_mul_ps(mat[1], vec);
		res[2] = _mm_mul_ps(mat[2], vec);

		res[0] = _mm_hadd_ps(res[0], res[0]);
		res[0] = _mm_hadd_ps(res[0], res[0]);

		res[1] = _mm_hadd_ps(res[1], res[1]);
		res[1] = _mm_hadd_ps(res[1], res[1]);
		
		res[2] = _mm_hadd_ps(res[2], res[2]);
		res[2] = _mm_hadd_ps(res[2], res[2]);

		_mm_store_ss(&ret->o.x, res[0]);
		_mm_store_ss(&ret->o.y, res[1]);
		_mm_store_ss(&ret->o.z, res[2]);				 	 	
	
		// Ray tmp;

		//tmp.o = (*this)(r.o);
		// tmp.d = (*this)(r.d);

		/*
		if( tmp.d.x != ret->d.x )
		{
			cout << "tmp.x " << tmp.d.x  << " sse3.x " << ret->d.x << endl;
			exit(0);
		}

		if( tmp.d.y != ret->d.y )
		{
			cout << "tmp.y " << tmp.d.y  << " sse3.y " << ret->d.y << endl;
			exit(0);
		}

		if( tmp.d.z != ret->d.z )
		{
			cout << "tmp.z " << tmp.d.z  << " sse3.z " << ret->d.z << endl;
			exit(0);
		}
		*/

		/*
		ret->o.x = tmp.o.x;
		ret->o.y = tmp.o.y;
		ret->o.z = tmp.o.z;
		*/
		/*
		ret->d.x = tmp.d.x;
		ret->d.y = tmp.d.y;
		ret->d.z = tmp.d.z;
		*/

		// ret->d.z = -ret->d.z;

		//ret->maxt = tmp.maxt;
		// ret->mint = tmp.mint;

	#else
	
		Ray tmp;

		tmp.o = (*this)(r.o);
		tmp.d = (*this)(r.d);

		ret->o.x = tmp.o.x;
		ret->o.y = tmp.o.y;
		ret->o.z = tmp.o.z;

		ret->d.x = tmp.d.x;
		ret->d.y = tmp.d.y;
		ret->d.z = tmp.d.z;

		ret->maxt = tmp.maxt;
		ret->mint = tmp.mint;

	#endif
	
		



}

void Transform::Print()
{
	cout << "Matrix" << endl;

	matrix->Print();

	cout << "Inverse Matrix" << endl;

	invMatrix->Print();

}	


Transform lzmath::Mult(const Transform &t1, const Transform &t2)
{
	return t1 * t2;
}


Transform lzmath::RotateX(float a)
{
	Matrix4x4f *matrix;

	a *= (float)M_PI/180.f;	

	float sin_f = sinf(a);
	float cos_f = cosf(a);

	matrix = new Matrix4x4f(	1, 0, 0, 0,
								0, cos_f, -sin_f, 0,
								0, sin_f, cos_f, 0,
								0, 0, 0, 1
							);	

	return Transform(matrix, matrix->Transpose());
}


Transform lzmath::RotateY(float a)
{
	Matrix4x4f *matrix;

	a *= (float)M_PI/180.f;

	float sin_f = sinf(a);
	float cos_f = cosf(a);

	matrix = new Matrix4x4f(	cos_f, 0, sin_f, 0,
								0, 1, 0, 0,
								-sin_f, 0, cos_f, 0,
								0, 0, 0, 1
							);	

	return Transform(matrix, matrix->Transpose());

}

Transform lzmath::RotateZ(float a)
{
	Matrix4x4f *matrix;

	a *= (float)M_PI/180.f;

	float sin_f = sinf(a);
	float cos_f = cosf(a);

	matrix = new Matrix4x4f(	cos_f, -sin_f, 0, 0,
								sin_f, cos_f, 0, 0,
								0, 0, 1, 0,
								0, 0, 0, 1
							);	

	return Transform(matrix, matrix->Transpose());

}

Transform lzmath::Rotate(float ang, const Vector3f &axis)
{
	Matrix4x4f *matrix;

	Vector3f a = Normalize(axis);

	ang *= (float)M_PI/180.f;

	float s = sinf(ang);
	float c = cosf(ang);
	
	float m[4][4];

	m[0][0] = a.x * a.x + (1.f - a.x * a.x) * c;
	m[0][1] = a.x * a.y * (1.f - c) - a.z * s;
	m[0][2] = a.x * a.z * (1.f - c) + a.y * s;
	m[0][3] = 0;

	m[1][0] = a.x * a.y * (1.f - c) + a.z * s;
	m[1][1] = a.y * a.y + (1.f - a.y * a.y) * c;
	m[1][2] = a.y * a.z * (1.f - c) - a.x * s;
	m[1][3] = 0;

	m[2][0] = a.x * a.z * (1.f - c) - a.y * s;
	m[2][1] = a.y * a.z * (1.f - c) + a.x * s;
	m[2][2] = a.z * a.z + (1.f - a.z * a.z) * c;
	m[2][3] = 0;

	m[3][0] = 0;
	m[3][1] = 0;
	m[3][2] = 0;
	m[3][3] = 1;

	matrix = new Matrix4x4f(m);

	return Transform(matrix, matrix->Transpose());
}



Transform lzmath::Translate(float x, float y, float z)
{
	Matrix4x4f *matrix, *invMatrix;

	matrix = new Matrix4x4f(	1, 0, 0, x,
								0, 1, 0, y,
								0, 0, 1, z,
								0, 0, 0, 1
							);


	invMatrix = new Matrix4x4f(	1, 0, 0, -x,
								0, 1, 0, -y,
								0, 0, 1, -z,
								0, 0, 0, 1
							);


	return Transform(matrix, invMatrix);
}


Transform lzmath::Scale(float x, float y, float z)
{
	Matrix4x4f *matrix, *invMatrix;

	matrix = new Matrix4x4f(	x, 0, 0, 0,
								0, y, 0, 0,
								0, 0, z, 0,
								0, 0, 0, 1 );


	invMatrix = new Matrix4x4f(	1.0f/x, 0, 0, 0,
								0, 1.0f/y, 0, 0,
								0, 0, 1.0f/z, 0,
								0, 0, 0, 1 );	

	return Transform (matrix, invMatrix);
}

// Camera space - world space
Transform lzmath::LookAt(const Point3f &pos, const Point3f &look, const Vector3f &up)
{
	Matrix4x4f *matrix;

	float m[4][4];
	
	Vector3f dir = Normalize(look - pos);
	Vector3f right = Cross(dir, Normalize(up));
	Vector3f newUp = Cross(right, dir);
	
	m[0][0] = right.x;
	m[1][0] = right.y;
	m[2][0] = right.z;
	m[3][0] = 0.;
	m[0][1] = newUp.x;
	m[1][1] = newUp.y;
	m[2][1] = newUp.z;
	m[3][1] = 0.;
	m[0][2] = dir.x;
	m[1][2] = dir.y;
	m[2][2] = dir.z;
	m[3][2] = 0.;
	m[0][3] = pos.x;
	m[1][3] = pos.y;
	m[2][3] = pos.z;
	m[3][3] = 1;

	matrix = new Matrix4x4f(m); // cam2world

	return Transform(matrix->Inverse(), matrix);
}

Transform lzmath::Perspective(float fov, float n, float f)
{
	float val = 1.0f / (f-n);

	Matrix4x4f *mat = new Matrix4x4f(	1, 0, 0, 0,
										0, 1, 0, 0,
										0, 0, f*val, -f*n*val,
										0, 0, 1, 0);		


	float invTanAng = 1.0f / tanf(fov/ 2.0f);

	
	return Scale(invTanAng, invTanAng, 1.0f) * Transform(mat);

}




