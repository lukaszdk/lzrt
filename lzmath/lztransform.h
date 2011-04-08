#ifndef _LZMATH_LZTRANSFORM_H_
#define _LZMATH_LZTRANSFORM_H_



namespace lzmath
{
	class Transform;
	class Matrix4x4f;
	class Vector3f;
	class Point3f;
	class Normal;
	class Ray;

	Transform RotateX(float a);
	Transform RotateY(float a);
	Transform RotateZ(float a);
	Transform Rotate(float a, const Vector3f &axis);
	Transform Translate(float x, float y, float z);
	Transform Scale(float x, float y, float z);
	Transform LookAt(const Point3f &pos, const Point3f &look, const Vector3f &up);
	Transform Perspective(float fov, float n, float f);
	Transform Mult(const Transform &t1, const Transform &t2);


}

class lzmath::Transform
{
	public:
		Matrix4x4f *matrix;
		Matrix4x4f *invMatrix;
	public:
		Transform();
		Transform(Matrix4x4f *matrix);
		Transform(Matrix4x4f *matrix, Matrix4x4f *invMatrix); 
		Transform operator*(const Transform &t) const;
		Vector3f operator()(const Vector3f &v) const;
		Point3f operator()(const Point3f &p) const;
		void operator()(Point3f *ret, const Point3f &p) const;
		void operator()(float *ret, float p[3]) const;
		Normal operator()(const Normal &n) const;
		Ray operator()(const Ray &r) const;
		void operator()(Ray *ret, const Ray &r) const;
		Transform GetInverse() const;
		void ApplyNormal(float n[3], float ret[3]);
		void Print();
};


#endif

