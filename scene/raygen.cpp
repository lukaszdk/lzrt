#include <mesh/aabb.h>
#include <scene/math3d.h>
#include <scene/raytracejob.h>
#include <scene/scene.h>
#include <math.h>
#include <stdio.h>

#ifdef _CELL
	#include <spu/kernels/raygen.h>
	#include <spu/kernels/polytest.h>
	#include <spu/speprogram.h>

	#include <vec_types.h>
	#include <altivec.h>
	#include <spu2vmx.h>
	#include <xform_vec3.h>
	#include <xform_vec4.h>
	#include <transpose_matrix4x4.h>
	#include <normalize3.h>
	#include <malloc_align.h>
	#include <free_align.h>
	#include <cross_product3.h>
	#include <dot_product3.h>
	#include <max_vec_float3.h>
	#include <min_vec_float3.h>
	#include <max_vec_float4.h>
	#include <min_vec_float4.h>
	#include <sum_across_float4.h>
#endif

// SPE Kernels
#ifdef _CELL
extern spe_program_handle_t raygen;
#endif

extern kdnode_t* KDGetNodes(int *nnodes = 0);
extern int KDGetNumLeaves();
aabb_t *KDGetWorldAABB();

/*
static int dep; // frustum deepest entry point
static int total_dep;
static int nfrustums;
static int nfrustumhits;
*/

using namespace std;

float raster2cam[4][4] ALIGNED(16);
float cam2world[4][4] ALIGNED(16);

/*
int GetFrustrumDEP()
{
	return dep;
}

int GetFrustumTotalDEP()
{
	return total_dep;
}

int GetNumFrustrum()
{
	return nfrustums;
}

int GetNumFrustrumHits()
{
	return nfrustumhits;
}
*/


#ifdef _CELL
void RayTraceJob::RayGenSPU(int startx, int starty, int endx, int endy)
{
	unsigned int a_nfrustums ALIGNED(16);
	unsigned int a_nfrustumhits ALIGNED(16);
	unsigned int a_frustum_dep ALIGNED(16);
	unsigned int a_total_frustum_dep ALIGNED(16);

	a_nfrustums = 0;
	a_nfrustumhits = 0;
	a_frustum_dep = 0;
	a_total_frustum_dep = 0;

	buffer_polypartition.Clear();

	ResetPolyPartition();

	int numhits ALIGNED(16);

	float raster2cam[4][4] ALIGNED(16);
	float cam2world[4][4] ALIGNED(16);

	GetScene()->camera->GetRaster2Cam(raster2cam);
	GetScene()->camera->GetCam2World(cam2world);

	// Setup argument
	raygen_arg_t arg ALIGNED(16);
	memcpy(arg.raster2cam, raster2cam, sizeof(float[4][4]));
	memcpy(arg.cam2world, cam2world, sizeof(float[4][4]));
	arg.startx = startx;
	arg.starty = starty;
	arg.endx = endx;
	arg.endy = endy;
	arg.numrays =  buffer_polypartition.size;
	arg.buffer_polypartition = buffer_polypartition.CopyToPtr();
	arg.znear = GetScene()->camera->znear;
	arg.zfar = GetScene()->camera->zfar;

	memcpy(&arg.worldaabb, KDGetWorldAABB(), sizeof(aabb_t));	
	arg.nodes = KDGetNodes(&arg.numnodes);
	
	arg.count = count;
	arg.numleaves = KDGetNumLeaves();
	arg.numhits = &numhits;
	arg.nfrustums = &a_nfrustums;
	arg.nfrustumhits = &a_nfrustumhits;
	arg.frustum_dep = &a_frustum_dep;
	arg.total_frustum_dep = &a_total_frustum_dep;

	
	if(GetScene()->raygen_kernel == K_RAYGEN_S_SIMD_SPU || GetScene()->raygen_kernel == K_RAYGEN_F_SIMD_SPU)
		arg.simd = 1;
	else
		arg.simd = 0;

	frustum_t f ALIGNED(16);

	int frustumdim = GetScene()->frustumdim;

	if(GetScene()->raygen_kernel == K_RAYGEN_FRUSTUM_SPU || GetScene()->raygen_kernel == K_RAYGEN_F_SIMD_SPU)
	{
		int width = endx - startx;
		int height = endy - starty;

		if( (width%frustumdim) > 0 || (height%frustumdim) > 0)
		{
			cout << "RayGenFrustum: Fatal error area(" << startx << ", " << starty;
			cout << ", " << endx << ", " << endy << ") not dividable with frustum ";
			cout << frustumdim << "x" << frustumdim << endl;
			exit(0);
		}


		arg.frustumdim = frustumdim;
		arg.frustumstep = GetScene()->frustumstep;
		arg.frustumstepsize = GetScene()->frustumstepsize;
		arg.frustum = 1;
	}
	else
	{
		arg.frustumdim = 0;
		arg.frustum = 0;
	}


	// Load and Run SPE program
	SPEProgram spe_raygen(&raygen);
	spe_raygen.Run(GetSPEContext(tid), &arg);	


	//cout << "a_nfrustums " << a_nfrustums << endl;
	//cout << "a_nfrustumhits " << a_nfrustumhits << endl;
	//cout << "a_frustum_dep " << a_frustum_dep << endl;
	//cout << "a_total_frustum_dep " << a_total_frustum_dep << endl;

	nfrustums += a_nfrustums;
	nfrustumhits += a_nfrustumhits;

	if(a_frustum_dep > frustum_dep ) frustum_dep = a_frustum_dep;
	
	total_frustum_dep += a_total_frustum_dep;

	nleafhits += numhits;
	nleafvisits += numhits;

	buffer_polypartition.Increment(numhits);	
}
#endif


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

inline void IntersectAABB(aabb_t *aabb, float *o, float *d,  float *tmin, float *tmax, int axis)
{
	//if(d[axis] == 0) d[axis] = 0.000001f;

	float invRayDir;
 
	if(d[axis] == 0) 
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

#ifdef _CELL
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
#endif

bool IntersectFrustum(aabb_t *aabb, frustum_t *f)
{
	float ix[4][2];
	float iy[4][2];
	float iz[4][2];

	for(int i=0; i < 4; i++)
	{
		IntersectAABB(aabb, f->o[i], f->d[i], &ix[i][0], &ix[i][1], 0);
		IntersectAABB(aabb, f->o[i], f->d[i], &iy[i][0], &iy[i][1], 1);
	}

	float x_min = min4(ix[0][0], ix[1][0], ix[2][0], ix[3][0]);
	float x_max = max4(ix[0][1], ix[1][1], ix[2][1], ix[3][1]);

	float y_min = min4(iy[0][0], iy[1][0], iy[2][0], iy[3][0]);
	float y_max = max4(iy[0][1], iy[1][1], iy[2][1], iy[3][1]);

	if(y_min > x_max || x_min > y_max) return false;

	for(int i=0; i < 4; i++)
		IntersectAABB(aabb, f->o[i], f->d[i], &iz[i][0], &iz[i][1], 2);

	float z_min = min4(iz[0][0], iz[1][0], iz[2][0], iz[3][0]);
	float z_max = max4(iz[0][1], iz[1][1], iz[2][1], iz[3][1]);

	if(y_min > z_max || z_min > y_max) return false;
	if(x_min > z_max || z_min > x_max) return false;

	return true;
}

static inline int IsLeaf(kdnode_t *node)
{
	return !node->left;
}

int RayTraceJob::FindEntryPoint(frustum_t *f, kdnode_t *nodes, aabb_t *worldaabb)
{
	if(!IntersectFrustum(worldaabb, f)) return -1;

	// AOS -> SOA
	MatrixTranspose(f->o);
	MatrixTranspose(f->d);

	aabb_t aabb;
	
	memcpy(&aabb, worldaabb, sizeof(aabb_t));

	int ep = 0;	

	int axis = nodes[0].axis;
	float split = nodes[0].split;

	float sign[3];

	int ncount[3];
	int pcount[3];

	for(int a=0; a < 3; a++)
	{
		ncount[a] = 0;
		pcount[a] = 0;

		sign[a] = 0;

		for(int i=0; i < 4; i++)
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


		// cout << "ep: " << ep << endl;
		_dep++;

		last_ep = ep;

		axis = nodes[ep].axis;
		split = nodes[ep].split;
		
		for(int i=0; i < 4; i++)
			t[i] = (split - f->o[axis][i]) / f->d[axis][i];

		bool first = f->o[axis][0] <= split;
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
				
			
			for(int a=0; a < 2 && last_ep == ep; a++)
			{
				for(int i=0; i < 4; i++)
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
				if(ep == nodes[last_ep].left)
				{
					aabb.max[ axis ] = split;
				}
				else
				{
					aabb.min[ axis ] = split;
				}
			}

		}

		if(_dep > frustum_dep ) frustum_dep = _dep;

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

#ifdef _CELL

vector float vone = spu_splats(1.0f);
vector float vzero = spu_splats(0.0f);

int RayTraceJob::FindEntryPointSIMD(frustum_t *f, kdnode_t *nodes, aabb_t *worldaabb)
{
	if(!IntersectFrustumSIMD(worldaabb, f)) return -1;

	vector float *ray_d = (vector float*)f->d;
	vector float *ray_o = (vector float*)f->o;

	// AOS -> SOA
	_transpose_matrix4x4(ray_o, ray_o);
	_transpose_matrix4x4(ray_d, ray_d);

	aabb_t aabb;
	
	memcpy(&aabb, worldaabb, sizeof(aabb_t));

	int ep = 0;	

	int axis = nodes[0].axis;
	float split = nodes[0].split;

	float sign[3];

	float ncount[3];
	float pcount[3];
	
	for(int a=0; a < 3; a++)
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

		_dep++;

		last_ep = ep;

		axis = nodes[ep].axis;
		split = nodes[ep].split;
		
		vector float vsplit = spu_splats(nodes[ep].split);

		t = spu_mul( spu_sub(vsplit, ray_o[axis]), invdir[axis]);

		bool first = f->o[axis][0] <= split;
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
				
			
			for(int a=0; a < 2 && last_ep == ep; a++)
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
				if(ep == nodes[last_ep].left)
				{
					aabb.max[ axis ] = split;
				}
				else
				{
					aabb.min[ axis ] = split;
				}
			}

		}

		if(_dep > frustum_dep ) frustum_dep = _dep;

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
#endif


inline void GenRay(float *o, float *d, float x, float y)
{
	float p_raster[4] ALIGNED(16) = {x, y, 0, 1};
				
	#ifdef _CELL
	if(GetScene()->raygen_kernel == K_RAYGEN_F_SIMD)
	{
		MultMatrixPointSIMD(raster2cam, p_raster);
		NormalizeSIMD(p_raster);
	}
	else
	#endif
	{
		MultMatrixPoint(raster2cam, p_raster);
		Normalize(p_raster);
	}

	o[0] = 0;
	o[1] = 0;
	o[2] = 0;
	d[0] = p_raster[0];
	d[1] = p_raster[1];
	d[2] = p_raster[2];

	#ifdef _CELL
	if(GetScene()->raygen_kernel == K_RAYGEN_F_SIMD)
	{	
		MultMatrixPointSIMD(cam2world, o);
		MultMatrixVectorSIMD(cam2world, d);
	}
	else
	#endif
	{
		MultMatrixPoint(cam2world, o);
		MultMatrixVector(cam2world, d);
	}


}

inline void CreateFrustum(frustum_t *f, float x, float y, float dim)
{
	GenRay(f->o[0], f->d[0], x, y);
	GenRay(f->o[1], f->d[1], x+dim, y);
	GenRay(f->o[2], f->d[2], x, y+dim);
	GenRay(f->o[3], f->d[3], x+dim, y+dim);
}

void RayTraceJob::RayGenFrustumRecursive(int x, int y, int frustumdim, int frustumstep)
{
	frustum_t f ALIGNED(16);

	CreateFrustum(&f, x, y, frustumdim);

	nfrustums++;

	int ep;

	#ifdef _CELL
	if(GetScene()->raygen_kernel == K_RAYGEN_F_SIMD)
		ep = FindEntryPointSIMD(&f, KDGetNodes(), KDGetWorldAABB());
	else
	#endif
		ep = FindEntryPoint(&f, KDGetNodes(), KDGetWorldAABB());			

	if(ep >= 0) nfrustumhits++;

	int new_frustumdim;
	
	if(GetScene()->frustumstepsize > 0) 
		new_frustumdim = frustumdim / (2*GetScene()->frustumstepsize);
	else
		new_frustumdim = frustumdim;

	if(frustumdim < 4 || frustumstep == 0 || new_frustumdim < 2)
	{
		if(ep >= 0)
		{
			RayGenSingle(x,y, x+frustumdim, y+frustumdim, ep);
		}
	}
	else
	{
		for(int x2=x; x2 < x+frustumdim; x2+=new_frustumdim)
		{
			for(int y2=y; y2 < y+frustumdim; y2+=new_frustumdim)
			{
				RayGenFrustumRecursive(x2, y2, new_frustumdim, frustumstep-1);
			}
		}	
	}
}


void RayTraceJob::RayGenFrustum(int startx, int starty, int endx, int endy)
{
	buffer_polypartition.Clear();

	int width = endx - startx;
	int height = endy - starty;

	int frustumdim = GetScene()->frustumdim;
	int frustumstep = GetScene()->frustumstep;

	if( (width%frustumdim) > 0 || (height%frustumdim) > 0)
	{
		cout << "RayGenFrustum: Fatal error area(" << startx << ", " << starty;
		cout << ", " << endx << ", " << endy << ") not dividable with frustum ";
		cout << frustumdim << "x" << frustumdim << endl;
		exit(0);
	}

	frustum_t f ALIGNED(16);

	for(int x=startx; x < endx; x+=frustumdim)
	{
		for(int y=starty; y < endy; y+=frustumdim)
		{
			/*
			CreateFrustum(&f, x, y, frustumdim);
	
			int ep;
	
			#ifdef _CELL
			if(raygen_kernel == K_RAYGEN_F_SIMD)
				ep = FindEntryPointSIMD(&f, KDGetNodes(), KDGetWorldAABB());
			else
			#endif
				ep = FindEntryPoint(&f, KDGetNodes(), KDGetWorldAABB());			

			if(ep >= 0)
			{
				RayGenSingle(x,y, x+frustumdim, y+frustumdim, ep);
			}
			*/
			
			RayGenFrustumRecursive(x, y, frustumdim, frustumstep);


		}
	}
}

void RayTraceJob::RayGen(int startx, int starty, int endx, int endy)
{
	GetScene()->camera->GetRaster2Cam(raster2cam);
	GetScene()->camera->GetCam2World(cam2world);

	ResetPolyPartition();

	int raygen_kernel = GetScene()->raygen_kernel;

	frustum_dep = 0;
	nfrustums = 0;
	nfrustumhits = 0;
	total_frustum_dep = 0;

	#ifdef _CELL
	if(raygen_kernel == K_RAYGEN_FRUSTUM || raygen_kernel == K_RAYGEN_F_SIMD)
	#else
	if(raygen_kernel == K_RAYGEN_FRUSTUM)
	#endif
		RayGenFrustum(startx, starty, endx, endy);
	else
		RayGenSingle(startx, starty, endx, endy);
}

void RayTraceJob::RayGenSingle(int startx, int starty, int endx, int endy, int ep)
{
	int x = startx;
	int y = starty;
	
	raystate_t *rays = (raystate_t*)buffer_polypartition.CopyToPtr(); 
	int numrays = (endx-startx)*(endy-starty);

	int j = 0;

	Camera *camera = GetScene()->camera;

	for(int r=0; r < numrays; r++)
	{
		float p_raster[4] ALIGNED(16) = {x, y, 0, 1};
				
		#ifdef _CELL
		if(GetScene()->raygen_kernel == K_RAYGEN_S_SIMD)
		{
			MultMatrixPointSIMD(raster2cam, p_raster);
			NormalizeSIMD(p_raster);
		}
		else
		#endif
		{
			MultMatrixPoint(raster2cam, p_raster);
			Normalize(p_raster);
		}

		rays[j].ray.o[0] = 0;
		rays[j].ray.o[1] = 0;
		rays[j].ray.o[2] = 0;
		rays[j].ray.d[0] = p_raster[0];
		rays[j].ray.d[1] = p_raster[1];
		rays[j].ray.d[2] = p_raster[2];

		#ifdef _CELL
		if(GetScene()->raygen_kernel == K_RAYGEN_S_SIMD)
		{	
			MultMatrixPointSIMD(cam2world, rays[j].ray.o);
			MultMatrixVectorSIMD(cam2world, rays[j].ray.d);
		}
		else
		#endif
		{
			MultMatrixPoint(cam2world, rays[j].ray.o);
			MultMatrixVector(cam2world, rays[j].ray.d);
		}

		rays[j].ray.tmin = 0;
		rays[j].ray.tmax = (camera->zfar - camera->znear);
		rays[j].x = x;
		rays[j].y = y;
		rays[j].depth = 0;
		rays[j].phit = 0;
		rays[j].leaf = -1;
		rays[j].leafcount = 0;
		rays[j].ep = ep;
		rays[j].mode = RAY_MODE_PRIMARY;
		
		x++;
		if(x == endx) 
		{
			x = startx;
			y++;			
		}

		FindLeaf(&rays[j], KDGetNodes(), KDGetWorldAABB(), tid);	
		
		if(rays[j].leaf >= 0) 
		{
			PolyPartitionCount(&rays[j]);
			j++;
		}

	}

	nleafhits += j;
	nleafvisits += j;

	buffer_polypartition.Increment(j);

}
