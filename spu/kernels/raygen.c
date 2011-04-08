#include <stdio.h>
#include <raygen.h>
#include <kdtravers.h>
#include <spu_intrinsics.h>
#include <spu_mfcio.h>
#include <xform_vec3.h>
#include <xform_vec4.h>
#include <transpose_matrix4x4.h>
#include <normalize3.h>
#include <max_vec_float3.h>
#include <min_vec_float3.h>
#include <max_vec_float4.h>
#include <min_vec_float4.h>
#include <sum_across_float4.h>
#include <math.h>
#include <dma.h>
#include <doublebuf.h>
#include <polypartition.h>

#define BUFFER_SIZE		4*1024
#define NUM_RAYS		(BUFFER_SIZE/sizeof(raystate_t))

raygen_arg_t arg;

raystate_t ray_buffer[2][NUM_RAYS] ALIGNED(16);
kdnode_t nodes[ KD_NUMNODES ] ALIGNED(16);

count_t count[NUM_LEAVES] ALIGNED(16);

vector float raster2cam[4];
vector float cam2world[4];

float fraster2cam[4][4] ALIGNED(16);
float fcam2world[4][4] ALIGNED(16);

int totalhits ALIGNED(16);

unsigned int nfrustums ALIGNED(16);
unsigned int nfrustumhits ALIGNED(16);
unsigned int frustum_dep ALIGNED(16);
unsigned int total_frustum_dep ALIGNED(16);


doublebuf_t ray_db;
struct mfc_list_element list[16] ALIGNED(16);

static kdstack_t stack[32];

void ResetCount()
{
	int i;

	for(i=0; i < arg.numleaves; i++)
	{
		count[i].value = 0;
		count[i].offset = 0;
		count[i].ptr = 0;
	}	
}

void LeafCount(raystate_t *ray)
{
	count[ray->leaf].value++;
}


inline void MultMatrixVector3SIMD(vector float matrix[4], float v[4])
{
	v[3] = 0;
	vector float *vv = (vector float *)v;
	vector float rv = _xform_vec4(*vv, matrix);	
	*vv = rv;
}

inline void MultMatrixVector4SIMD(vector float matrix[4], float v[4])
{
	vector float *vv = (vector float *)v;
	vector float rv = _xform_vec4(*vv, matrix);
	*vv = rv;
}

inline void NormalizeSIMD(float v[4])
{
	vector float *vv = (vector float*)v;
	*vv = _normalize3(*vv);
}

inline void MultMatrixVector4(float matrix[4][4], float v[4])
{
	float ret[4] ALIGNED(16);
	
	ret[0] = matrix[0][0]*v[0] + matrix[0][1]*v[1] + matrix[0][2]*v[2] + matrix[0][3];
	ret[1] = matrix[1][0]*v[0] + matrix[1][1]*v[1] + matrix[1][2]*v[2] + matrix[1][3];
	ret[2] = matrix[2][0]*v[0] + matrix[2][1]*v[1] + matrix[2][2]*v[2] + matrix[2][3];
	
	v[0] = ret[0];
	v[1] = ret[1];
	v[2] = ret[2];
}

inline void MultMatrixVector3(float matrix[4][4], float v[3])
{
	float ret[4] ALIGNED(16);
	
	ret[0] = matrix[0][0]*v[0] + matrix[0][1]*v[1] + matrix[0][2]*v[2];
	ret[1] = matrix[1][0]*v[0] + matrix[1][1]*v[1] + matrix[1][2]*v[2];
	ret[2] = matrix[2][0]*v[0] + matrix[2][1]*v[1] + matrix[2][2]*v[2];

	v[0] = ret[0];
	v[1] = ret[1];
	v[2] = ret[2];
}

void MatrixTranspose2(float m[4][4], float ret[4][4])
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

void MatrixTranspose(float m[4][4])
{
	float ret[4][4];

	MatrixTranspose2(m, ret);

	int i,j;

	for(i=0; i < 4; i++)
		for(j=0; j < 4; j++)
			m[i][j] = ret[i][j];
}


inline void Normalize(float v[4])
{
	float len = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
	float d = 1 / len;	

	v[0] *= d;
	v[1] *= d;
	v[2] *= d;
}

inline vector float spu_max(vector float a, vector float b)
{
	return spu_sel( b, a, spu_cmpgt( a, b ) );
}

inline vector float spu_min(vector float a, vector float b)
{
	return spu_sel( a, b, spu_cmpgt( a, b ) );
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


int Intersect(volatile aabb_t *aabb, ray_t *ray, float *tmin, float *tmax)
{
	vector float vray_o = *(vector float*)ray->o;
	vector float vray_d = *(vector float*)ray->d;
	vector float vaabb_min = *(vector float*)aabb->min;
	vector float vaabb_max = *(vector float*)aabb->max;

	vector float vt0 = spu_splats(ray->tmin);
	vector float vt1 = spu_splats(ray->tmax);

	vector float invraydir = spu_re(vray_d);
	vector float vp = spu_mul(vray_o, invraydir);
	
	vector float _tNear = spu_msub( vaabb_min, invraydir, vp);
	vector float _tFar = spu_msub( vaabb_max, invraydir, vp);

	vector float tNear = spu_min(_tNear, _tFar);
	vector float tFar = spu_max(_tNear, _tFar);

	vt0 = spu_max(tNear, vt0);
	vt1 = spu_min(tFar, vt1);

	*tmin = _max_vec_float3(vt0);
	*tmax = _min_vec_float3(vt1);

	return !(*tmin > *tmax);
}

inline int IsLeaf(kdnode_t *node)
{
	return !node->left;
}


void FindLeaf(raystate_t *ray, kdnode_t *nodes, volatile aabb_t *worldaabb)
{
	float tmin, tmax;
	
	if(ray->leafcount == 0)
	{
		ray->ep = 0;

		if(!Intersect(worldaabb, &ray->ray, &tmin, &tmax))
		{	
			ray->leaf = -1;
			return;
		}
		else
		{
			ray->ray.tmin = max(tmin, ray->ray.tmin);
			ray->ray.tmax = min(tmax, ray->ray.tmax);
		}
	}
	else
	{
		ray->ray.tmin = ray->org_tmin;
		ray->ray.tmax = ray->org_tmax;

		tmin = ray->org_tmin;
		tmax = ray->org_tmax;
	}
	
	int stackptr = 0;
	
	float invdir[4] ALIGNED(16);
	vector float *vinvdir = (vector float *)invdir;
	vector float vray_d = *(vector float*)ray->ray.d;

	*vinvdir = spu_re( vray_d );

	int n = ray->ep;
	int leafcount = 0;

	while(1)
	{
		if(IsLeaf(&nodes[n]))
		{
			if(ray->leafcount == leafcount) 
			{
				ray->org_tmin = ray->ray.tmin;
				ray->org_tmax = ray->ray.tmax;

				ray->ray.tmin = tmin;
				ray->ray.tmax = tmax;

				ray->leaf = nodes[n].leafid;
				ray->leafcount++;

				return;
			}

			if(stackptr == 0) 
			{
				ray->leaf = -1;
				return;
			}

			stackptr--;

			n = stack[stackptr].node;
			tmin = stack[stackptr].tmin;
			tmax = stack[stackptr].tmax;
		 
			leafcount++;
		}
		else
		{
			register float tplane = (nodes[n].split - ray->ray.o[nodes[n].axis]) * invdir[nodes[n].axis];
			int first = ray->ray.o[nodes[n].axis] <= nodes[n].split;

			int firstChild, secondChild;

			if(first)
			{	
				firstChild = nodes[n].left;
				secondChild = nodes[n].right;
			}
			else
			{
				firstChild = nodes[n].right;
				secondChild = nodes[n].left;
			}

			if(tplane > tmax || tplane < 0)
			{
				n = firstChild;
			}
			else 
			{

				if(tplane < tmin)
				{
					n = secondChild;
				}
				else
				{
					stack[stackptr].node = secondChild;
					stack[stackptr].tmin = tplane;
					stack[stackptr].tmax = tmax;
	
					stackptr++; 

					n = firstChild;
					tmax = tplane;
				}
			}
		}
	}
}

int RayGenSingle(raystate_t *rays, int numrays, float firstx, float firsty, int startx, int endx, int ep )
{
	int i=0;
	int r;

	int x = firstx;
	int y = firsty;

	vector float vz = { 0, 0, 0, 1} ;
	float tmax = (arg.zfar - arg.znear);

	if(arg.simd == 1)
	{
		for(r=0; r < numrays; r++)
		{
			float p_raster[4] ALIGNED(16) = {x, y, 0, 1};
				
			MultMatrixVector4SIMD(raster2cam, p_raster);
			NormalizeSIMD(p_raster);

			vector float *vray_o = (vector float*)rays[i].ray.o;
			vector float *vray_d = (vector float*)rays[i].ray.d;

			*vray_o = vz;		
			*vray_d = *((vector float*)p_raster);
		
			rays[i].x = x;
			rays[i].y = y;
			rays[i].depth = 0;
			rays[i].phit = 0;
			rays[i].leaf = -1;
			rays[i].leafcount = 0;
			rays[i].ep = ep;
			rays[i].mode = RAY_MODE_PRIMARY;

			MultMatrixVector4SIMD(cam2world, rays[i].ray.o);
			MultMatrixVector3SIMD(cam2world, rays[i].ray.d);
		
			rays[i].ray.tmin = 0;
			rays[i].ray.tmax = tmax;

			FindLeaf(&rays[i], nodes, &arg.worldaabb);

			if(rays[i].leaf >=0) 
			{
				LeafCount(&rays[i]);
				i++;
			}

			x++;

			if(x == endx) 
			{
				x = startx;
				y++;			
			}
		}

		return i;

	}
	else
	{
		for(r=0; r < numrays; r++)
		{
			float p_raster[4] ALIGNED(16) = {x, y, 0, 1};
				
			MultMatrixVector4(fraster2cam, p_raster);
			Normalize(p_raster);

			vector float *vray_o = (vector float*)rays[i].ray.o;
			vector float *vray_d = (vector float*)rays[i].ray.d;

			*vray_o = vz;		
			*vray_d = *((vector float*)p_raster);
		
			rays[i].x = x;
			rays[i].y = y;
			rays[i].depth = 0;
			rays[i].phit = 0;
			rays[i].leaf = -1;
			rays[i].leafcount = 0;
			rays[i].ep = ep;
			rays[i].mode = RAY_MODE_PRIMARY;

			MultMatrixVector4(fcam2world, rays[i].ray.o);
			MultMatrixVector3(fcam2world, rays[i].ray.d);
		
			rays[i].ray.tmin = 0;
			rays[i].ray.tmax = tmax;

			FindLeaf(&rays[i], nodes, &arg.worldaabb);

			if(rays[i].leaf >=0)
			{
				LeafCount(&rays[i]);
				i++;
			}

			x++;

			if(x == endx) 
			{
				

				x = startx;
				y++;			
			}
		}

		return i;
	}
}


float min4(float v1, float v2, float v3, float v4)
{
	float min = v1;

	if(v2 < min) min = v2;
	if(v3 < min) min = v3;
	if(v4 < min) min = v4;

	return min;
}

float max4(float v1, float v2, float v3, float v4)
{
	float max = v1;

	if(v2 > max) max = v2;
	if(v3 > max) max = v3;
	if(v4 > max) max = v4;

	return max;
}


void IntersectAABB(aabb_t *aabb, float *o, float *d,  float *tmin, float *tmax, int axis)
{
	float invRayDir;
 
	if(d[axis] == 0)  // SPU FPU fix
		invRayDir = 100000000.0f;
	else
		invRayDir = 1.0f / d[axis];


	float tNear = (aabb->min[axis] - o[axis]) * invRayDir;
	float tFar =  (aabb->max[axis] - o[axis]) * invRayDir;

	if(tNear > tFar)
	{
		float tmp = tNear;
		tNear = tFar;
		tFar = tmp;
	}

	*tmin = tNear;
	*tmax = tFar;
}

inline void IntersectAABBSIMD(aabb_t *aabb, vector float vray_o, vector float vray_d, vector float *tmin, vector float *tmax)
{
	vector float vaabb_min = *(vector float*)aabb->min;
	vector float vaabb_max = *(vector float*)aabb->max;

	vector float invraydir = spu_re(vray_d);
	vector float vp = spu_mul(vray_o, invraydir);
	
	vector float _tNear = spu_msub( vaabb_min, invraydir, vp);
	vector float _tFar = spu_msub( vaabb_max, invraydir, vp);

	*tmin = spu_min(_tNear, _tFar);
	*tmax = spu_max(_tNear, _tFar);
} 


int IntersectFrustumSIMD(aabb_t *aabb, frustum_t *f)
{
	vector float *ray_o = (vector float*)f->o;
	vector float *ray_d = (vector float*)f->d;

	vector float tmin[4];
	vector float tmax[4];

	IntersectAABBSIMD(aabb, ray_o[0], ray_d[0], &tmin[0], &tmax[0]);
	IntersectAABBSIMD(aabb, ray_o[1], ray_d[1], &tmin[1], &tmax[1]);
	IntersectAABBSIMD(aabb, ray_o[2], ray_d[2], &tmin[2], &tmax[2]);
	IntersectAABBSIMD(aabb, ray_o[3], ray_d[3], &tmin[3], &tmax[3]);

	_transpose_matrix4x4(tmin, tmin);
	_transpose_matrix4x4(tmax, tmax);

	float x_min = _min_vec_float4(tmin[0]);
	float x_max = _max_vec_float4(tmax[0]);

	float y_min = _min_vec_float4(tmin[1]);
	float y_max = _max_vec_float4(tmax[1]);

	if(y_min > x_max || x_min > y_max) return 0;	

	float z_min = _min_vec_float4(tmin[2]);
	float z_max = _max_vec_float4(tmax[2]);

	if(y_min > z_max || z_min > y_max) return 0;
	if(x_min > z_max || z_min > x_max) return 0;

	return 1;
}




int IntersectFrustum(aabb_t *aabb, frustum_t *f)
{
	float ix[4][2];
	float iy[4][2];
	float iz[4][2];

	int i;

	for(i=0; i < 4; i++)
	{
		IntersectAABB(aabb, f->o[i], f->d[i], &ix[i][0], &ix[i][1], 0);
		IntersectAABB(aabb, f->o[i], f->d[i], &iy[i][0], &iy[i][1], 1);
	}

	float x_min = min4(ix[0][0], ix[1][0], ix[2][0], ix[3][0]);
	float x_max = max4(ix[0][1], ix[1][1], ix[2][1], ix[3][1]);

	float y_min = min4(iy[0][0], iy[1][0], iy[2][0], iy[3][0]);
	float y_max = max4(iy[0][1], iy[1][1], iy[2][1], iy[3][1]);

	if(y_min > x_max || x_min > y_max) return 0;

	for(i=0; i < 4; i++)
		IntersectAABB(aabb, f->o[i], f->d[i], &iz[i][0], &iz[i][1], 2);

	float z_min = min4(iz[0][0], iz[1][0], iz[2][0], iz[3][0]);
	float z_max = max4(iz[0][1], iz[1][1], iz[2][1], iz[3][1]);

	if(y_min > z_max || z_min > y_max) return 0;
	if(x_min > z_max || z_min > x_max) return 0;

	return 1;
}


int FindEntryPoint(frustum_t *f, kdnode_t *nodes, aabb_t *worldaabb)
{
	if(IntersectFrustum(worldaabb, f) == 0) return -1;

	// AOS -> SOA
	MatrixTranspose(f->o);
	MatrixTranspose(f->d);

	aabb_t aabb ALIGNED(16);
	
	vector float *src = (vector float*)worldaabb;
	vector float *dst = (vector float*)&aabb;

	dst[0] = src[0];
	dst[1] = src[1];

	int ep = 0;	

	int axis = nodes[0].axis;
	float split = nodes[0].split;

	float sign[3];

	int ncount[3];
	int pcount[3];

	int a,i;

	for(a=0; a < 3; a++)
	{
		ncount[a] = 0;
		pcount[a] = 0;

		sign[a] = 0;

		for(i=0; i < 4; i++)
		{
			if(f->d[a][i] < 0) ncount[a]++;
				else
					if(f->d[a][i] > 0) pcount[a]++;
		}

		if(ncount[a] == 4) sign[a] = -1;
		if(pcount[a] == 4) sign[a] = 1;

		if(sign[a] == 0) return 0;		
	}

	float t[4];

	int last_ep = -1;

	int _dep = -1;

	total_frustum_dep--;

	while(ep != last_ep)
	{
		last_ep = ep;

		_dep++;

		axis = nodes[ep].axis;
		split = nodes[ep].split;
		
		for(i=0; i < 4; i++)
			t[i] = (split - f->o[axis][i]) / f->d[axis][i];

		int first = f->o[axis][0] <= split;
		int firstchild, secondchild;

		if(first)
		{
			firstchild = nodes[ep].left;
			secondchild = nodes[ep].right;
		}
		else
		{
			firstchild = nodes[ep].right;
			secondchild = nodes[ep].left;
		}

		if(t[0] < 0) 
		{
			ep = firstchild;
		}
		else
		{
			float val[4];
			float val_min, val_max;					

			int axis2[2];

			axis2[0] = (axis+1)%3;
			axis2[1] = (axis+2)%3;
				
			
			for(a=0; a < 2 && last_ep == ep; a++)
			{
				for(i=0; i < 4; i++)
					val[i] = f->o[axis2[a]][i] + f->d[axis2[a]][i] * t[i];

				val_min = min4(val[0], val[1], val[2], val[3]);
				val_max = max4(val[0], val[1], val[2], val[3]);

				if(val_max < aabb.min[axis2[a]])
				{
					if(sign[axis2[a]] < 0)		
						ep = firstchild;
					else		
						ep = secondchild;

				}		

				if(val_min > aabb.max[axis2[a]])
				{
					if(sign[axis2[a]] > 0)
						ep = firstchild;
					else		
						ep = secondchild;
				}

			}
	
			if(ep != last_ep)
			{
				if(ep == (int)nodes[last_ep].left)
				{
					aabb.max[ axis ] = split;
				}
				else
				{
					aabb.min[ axis ] = split;
				}
			}

		}

		total_frustum_dep++;


		if(ep < last_ep) return last_ep;
	}

	// If leaf, check robost	
	if(IsLeaf(&nodes[ep]))
	{
		if(!IntersectFrustum(&aabb, f)) return -1; 
	}

	return ep;
}

int FindEntryPointSIMD(frustum_t *f, kdnode_t *nodes, aabb_t *worldaabb)
{
	if(!IntersectFrustumSIMD(worldaabb, f)) return -1;

	vector float *ray_d = (vector float*)f->d;
	vector float *ray_o = (vector float*)f->o;

	// AOS -> SOA
	_transpose_matrix4x4(ray_o, ray_o);
	_transpose_matrix4x4(ray_d, ray_d);

	aabb_t aabb;
	
	vector float *src = (vector float*)worldaabb;
	vector float *dst = (vector float*)&aabb;

	dst[0] = src[0];
	dst[1] = src[1];

	int ep = 0;	

	int axis = nodes[0].axis;
	float split = nodes[0].split;

	float sign[3];

	float ncount[3];
	float pcount[3];
	
	int a;

	vector float vone = spu_splats(1.0f);
	vector float vzero = spu_splats(0.0f);

	for(a=0; a < 3; a++)
	{
		sign[a] = 0;
		
		vector unsigned int masklt0 = spu_cmpgt(vzero, ray_d[a]);
		vector unsigned int maskgt0 = spu_cmpgt(ray_d[a], vzero);

		ncount[a] = _sum_across_float4(spu_sel(vzero, vone, masklt0));
		pcount[a] = _sum_across_float4(spu_sel(vzero, vone, maskgt0));

		if(ncount[a] == 4) sign[a] = -1;
		if(pcount[a] == 4) sign[a] = 1;

		if(sign[a] == 0) return 0;		
	}

	vector float t;
	vector float invdir[3];

	invdir[0] = spu_re(ray_d[0]);
	invdir[1] = spu_re(ray_d[1]);
	invdir[2] = spu_re(ray_d[2]);

	int last_ep = -1;

	int _dep = -1;

	total_frustum_dep--;


	while(ep != last_ep)
	{
		// cout << "ep: " << ep << endl;

		last_ep = ep;
		_dep++;

		axis = nodes[ep].axis;
		split = nodes[ep].split;
		
		vector float vsplit = spu_splats(nodes[ep].split);

		t = spu_mul( spu_sub(vsplit, ray_o[axis]), invdir[axis]);

		int first = f->o[axis][0] <= split;
		int firstchild, secondchild;

		if(first)
		{
			firstchild = nodes[ep].left;
			secondchild = nodes[ep].right;
		}
		else
		{
			firstchild = nodes[ep].right;
			secondchild = nodes[ep].left;
		}

		if(t[0] < 0) 
		{
			ep = firstchild;
		}
		else
		{
			float val_min, val_max;					
			vector float val;
	
			int axis2[2];

			axis2[0] = (axis+1)%3;
			axis2[1] = (axis+2)%3;
				
			
			for(a=0; a < 2 && last_ep == ep; a++)
			{
				val = spu_madd(ray_d[axis2[a]], t, ray_o[axis2[a]]);
			
				val_min = _min_vec_float3(val);
				val_max = _max_vec_float3(val);


				if(val_max < aabb.min[axis2[a]])
				{
					if(sign[axis2[a]] < 0)		
						ep = firstchild;
					else		
						ep = secondchild;

				}		

				if(val_min > aabb.max[axis2[a]])
				{
					if(sign[axis2[a]] > 0)
						ep = firstchild;
					else		
						ep = secondchild;
				}

			}
	
			if(ep != last_ep)
			{
				if(ep == (int)nodes[last_ep].left)
				{
					aabb.max[ axis ] = split;
				}
				else
				{
					aabb.min[ axis ] = split;
				}
			}

		}


		if(_dep > (int)frustum_dep ) frustum_dep = _dep;

		total_frustum_dep++;

		if(ep < last_ep) return last_ep;
	}

	// If leaf, check robost	
	if(IsLeaf(&nodes[ep]))
	{
		if(!IntersectFrustumSIMD(&aabb, f)) return -1; 
	}

	return ep;
}

inline void GenRay(float *o, float *d, float x, float y)
{
	float p_raster[4] ALIGNED(16) = {x, y, 0, 1};
				
	if(arg.simd == 1)
	{
		MultMatrixVector4SIMD(raster2cam, p_raster);
		NormalizeSIMD(p_raster);
	}
	else
	{
		MultMatrixVector4(fraster2cam, p_raster);
		Normalize(p_raster);
	}

	o[0] = 0;
	o[1] = 0;
	o[2] = 0;
	o[3] = 1;
	d[0] = p_raster[0];
	d[1] = p_raster[1];
	d[2] = p_raster[2];
	d[3] = p_raster[3];

	if(arg.simd == 1)
	{	
		MultMatrixVector4SIMD(cam2world, o);
		MultMatrixVector3SIMD(cam2world, d);
	}
	else
	{
		MultMatrixVector4(fcam2world, o);
		MultMatrixVector3(fcam2world, d);
	}


}

inline void CreateFrustum(frustum_t *f, float x, float y, float dim)
{
	GenRay(f->o[0], f->d[0], x, y);
	GenRay(f->o[1], f->d[1], x+dim, y);
	GenRay(f->o[2], f->d[2], x, y+dim);
	GenRay(f->o[3], f->d[3], x+dim, y+dim);
}

void RayGenFrustumRecursive(int x, int y, int frustumdim, int frustumstep)
{
	frustum_t f ALIGNED(16);

	CreateFrustum(&f, x, y, frustumdim);

	nfrustums++;

	int ep;
	int size = 0;
	int numhits = 0;

	if(arg.simd)
		ep = FindEntryPointSIMD(&f,  nodes, &arg.worldaabb);
	else
		ep = FindEntryPoint(&f, nodes, &arg.worldaabb);			

	if(ep >= 0) nfrustumhits++;

	int new_frustumdim;
	
	if(arg.frustumstepsize > 0) 
		new_frustumdim = frustumdim / (2 * arg.frustumstepsize);
	else
		new_frustumdim = frustumdim;

	if(frustumdim < 4 || frustumstep == 0 || new_frustumdim < 2)
	{
		if(ep >= 0)
		{
			int frustumsize = frustumdim*frustumdim;

			//RayGenSingle(x,y, x+frustumdim, y+frustumdim, ep);

			int x2 = x;
			int y2 = y;
			int startx = x;
			int endx = x+arg.frustumdim;

			DoubleBufReset(&ray_db, frustumsize);

			while(!DoubleBufEmpty(&ray_db))
			{
				// Buffer 1
				DoubleBufWait(&ray_db, 0);
				size = DoubleBufDec(&ray_db, 0);
				numhits = RayGenSingle(ray_buffer[0], size, x2, y2, startx, endx, ep);
				DoubleBufPut(&ray_db, numhits, 0);
				totalhits += numhits;
	
				x2 += size;
				y2 += (x2 - startx)/(endx - startx);
				x2 = startx + ((x2 - startx) % (endx - startx));

				// Buffer 2
				DoubleBufWait(&ray_db, 1);
				size = DoubleBufDec(&ray_db, 1);
				numhits = RayGenSingle(ray_buffer[1], size, x2, y2, startx, endx, ep);
				DoubleBufPut(&ray_db, numhits, 1);
				totalhits += numhits;

				x2 += size;
				y2 += (x2 - startx)/(endx - startx);
				x2 = startx + ((x2 - startx) % (endx - startx));
			}		
		}
	}
	else
	{
		int x2, y2;

		for(x2=x; x2 < x+frustumdim; x2+=new_frustumdim)
		{
			for(y2=y; y2 < y+frustumdim; y2+=new_frustumdim)
			{
				RayGenFrustumRecursive(x2, y2, new_frustumdim, frustumstep-1);
			}
		}	
	}
}




int main(unsigned long long spu_id __attribute__ ((unused)), unsigned long long parm)
{
	unsigned int tag_id;
	unsigned int count_tag;

	tag_id = mfc_tag_reserve();
	count_tag = mfc_tag_reserve();

	// Transfer arg
	spu_mfcdma32(&arg, parm, sizeof(raygen_arg_t), tag_id, MFC_GET_CMD);
	DmaWait(tag_id);
		
	// Transfer kD-tree
	DmaGet(list, nodes, (unsigned int)arg.nodes, sizeof(kdnode_t)*arg.numnodes, tag_id);

	// Transpose matrices
	_transpose_matrix4x4(raster2cam, arg.raster2cam);
	_transpose_matrix4x4(cam2world, arg.cam2world);

	ResetCount();

	int x = arg.startx;
	int y = arg.starty;

	totalhits = 0;
	int numhits = 0;
	int size = 0;


	if(arg.simd == 0)
	{
		vector float *ptr = (vector float*)fraster2cam;
		ptr[0] = arg.raster2cam[0];
		ptr[1] = arg.raster2cam[1];
		ptr[2] = arg.raster2cam[2];
		ptr[3] = arg.raster2cam[3];

		ptr = (vector float*)fcam2world;
		ptr[0] = arg.cam2world[0];
		ptr[1] = arg.cam2world[1];
		ptr[2] = arg.cam2world[2];
		ptr[3] = arg.cam2world[3];
	}


	if(arg.frustum == 0)
	{
		DoubleBufInit(&ray_db, arg.numrays, arg.buffer_polypartition, sizeof(raystate_t), NUM_RAYS, ray_buffer[0], ray_buffer[1]);

		// Wait kdtree
		DmaWait(tag_id);

		while(!DoubleBufEmpty(&ray_db))
		{
			// Buffer 1
			DoubleBufWait(&ray_db, 0);
			size = DoubleBufDec(&ray_db, 0);
			numhits = RayGenSingle(ray_buffer[0], size, x, y, arg.startx, arg.endx, 0);
			DoubleBufPut(&ray_db, numhits, 0);
			totalhits += numhits;

			x += size;
			y += (x - arg.startx)/(arg.endx - arg.startx);
			x = arg.startx + ((x - arg.startx) % (arg.endx - arg.startx));

			// Buffer 
			DoubleBufWait(&ray_db, 1);
			size = DoubleBufDec(&ray_db, 1);
			numhits = RayGenSingle(ray_buffer[1], size, x, y, arg.startx, arg.endx, 0);
			DoubleBufPut(&ray_db, numhits, 1);
			totalhits += numhits;
	
			x += size;
			y += (x - arg.startx)/(arg.endx - arg.startx);
			x = arg.startx + ((x - arg.startx) % (arg.endx - arg.startx));
		}

		DmaPut(list, count, (uint)arg.count, sizeof(count_t) * arg.numleaves, count_tag);

		DoubleBufWait(&ray_db, 0);
		DoubleBufWait(&ray_db, 1);
	}
	else
	{
		
		int frustumsize = arg.frustumdim*arg.frustumdim;
		int x,y;

		// frustum_t f ALIGNED(16);

		DoubleBufInit(&ray_db, frustumsize, arg.buffer_polypartition, sizeof(raystate_t), NUM_RAYS, ray_buffer[0], ray_buffer[1]);
		
		// Wait kdtree
		DmaWait(tag_id);

		for(x=arg.startx; x < arg.endx; x+=arg.frustumdim)
		{
			for(y=arg.starty; y < arg.endy; y+=arg.frustumdim)
			{

				RayGenFrustumRecursive(x, y, arg.frustumdim, arg.frustumstep);

				/*
	
				CreateFrustum(&f, x, y, arg.frustumdim);

				int ep;

				if(arg.simd)
					ep = FindEntryPointSIMD(&f, nodes, &arg.worldaabb);
				else
					ep = FindEntryPoint(&f, nodes, &arg.worldaabb);

				if(ep >= 0)
				{
					int x2 = x;
					int y2 = y;
					int startx = x;
					int endx = x+arg.frustumdim;

					while(!DoubleBufEmpty(&ray_db))
					{
						// Buffer 1
						DoubleBufWait(&ray_db, 0);
						size = DoubleBufDec(&ray_db, 0);
						numhits = RayGenSingle(ray_buffer[0], size, x2, y2, startx, endx, ep);
						DoubleBufPut(&ray_db, numhits, 0);
						totalhits += numhits;
			
						x2 += size;
						y2 += (x2 - startx)/(endx - startx);
						x2 = startx + ((x2 - startx) % (endx - startx));

						// Buffer 2
						DoubleBufWait(&ray_db, 1);
						size = DoubleBufDec(&ray_db, 1);
						numhits = RayGenSingle(ray_buffer[1], size, x2, y2, startx, endx, ep);
						DoubleBufPut(&ray_db, numhits, 1);
						totalhits += numhits;
	
						x2 += size;
						y2 += (x2 - startx)/(endx - startx);
						x2 = startx + ((x2 - startx) % (endx - startx));
					}		

					DoubleBufReset(&ray_db, frustumsize);
				}
				*/
			}
		}
	
		DmaPut(list, count, (uint)arg.count, sizeof(count_t) * arg.numleaves, count_tag);	

		DoubleBufWait(&ray_db, 0);
		DoubleBufWait(&ray_db, 1);
	}

	
	/*
	unsigned int *nfrustums;
	unsigned int *nfrustumhits;
	unsigned int *frustum_dep;
	unsigned int *total_frustum_dep;
	*/

	spu_mfcdma32(&totalhits, (uint)arg.numhits, sizeof(int), tag_id, MFC_PUT_CMD);
	DmaWait(tag_id);
	spu_mfcdma32(&nfrustums, (uint)arg.nfrustums, sizeof(int), tag_id, MFC_PUT_CMD);
	DmaWait(tag_id);
	spu_mfcdma32(&nfrustumhits, (uint)arg.nfrustumhits, sizeof(int), tag_id, MFC_PUT_CMD);
	DmaWait(tag_id);
	spu_mfcdma32(&frustum_dep, (uint)arg.frustum_dep, sizeof(int), tag_id, MFC_PUT_CMD);
	DmaWait(tag_id);
	spu_mfcdma32(&total_frustum_dep, (uint)arg.total_frustum_dep, sizeof(int), tag_id, MFC_PUT_CMD);
	DmaWait(tag_id);



	

	DmaWait(count_tag);
	DmaWait(tag_id);

 	return 0;
}
