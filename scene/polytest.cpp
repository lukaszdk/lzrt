#include <scene/math3d.h>
#include <scene/raytracejob.h>
#include <scene/scene.h>
#include <math.h>

#ifdef _CELL
	#include <spu/kernels/raygen.h>
	#include <spu/kernels/polytest.h>
	#include <spu/kernels/polypartition.h>
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
	#include <length_vec3.h>
#endif

#ifdef _CELL
extern spe_program_handle_t polytest;
extern spe_program_handle_t polypartition;
#endif

extern void KDGetLeafPolys(int leaf, kdpoly_t **polys, int *numpolys);
extern int KDGetNumLeaves();
extern int KDGetMaxNumLeaves();
extern kdpolyp_t *KDGetPolys();
extern kdleafpoly_t *GetKDLeafPolysAll();

int KDGetNumPolys();


void MemorySwap(void *e1, void *e2, int size)
{
	if(size == 0) return;
	if(e1 == e2) return;

	unsigned char buffer[size];

	memcpy(buffer, e1, size);
	memcpy(e1, e2, size);
	memcpy(e2, buffer, size);
}

#ifdef _CELL
void RayTraceJob::PolyPartitionSPU(polytest_t **list, int *size)
{
	if(KDGetNumLeaves() > NUM_LEAVES)
	{
		cout << "Error: PolyPartitionSPU numleaves: " << KDGetNumLeaves() << " > " << NUM_LEAVES << endl;
		exit(0);
	}

	polypartition_arg_t arg ALIGNED(16);

	// Get KD Leaf poly pointers;
	int numleaves = KDGetNumLeaves();
	kdleafpoly_t *leafpolys = GetKDLeafPolysAll();

	/*
	printf("numleaves %i\n", numleaves);

	for(int i=0; i < numleaves; i++)
	{
		printf("leafpolys[%i].polys    = %08X\n", i, (uint)leafpolys[i].polys);
		printf("leafpolys[%i].numpolys = %08X\n", i, leafpolys[i].numpolys);
	}
	*/


	/*
	kdleafpoly_t *leafpolys = (kdleafpoly_t*)_malloc_align(sizeof(kdleafpoly_t) * numleaves, 4);
		
	for(int i=0; i < numleaves; i++)
	{
		KDGetLeafPolys(i, &leafpolys[i].polys, &leafpolys[i].numpolys);	
	}
	*/

	// Setup arg
	arg.numleaves = numleaves;
	arg.leafpolys = leafpolys;
	arg.rays = buffer_polypartition.buffer;
	arg.numrays = buffer_polypartition.NumElements();
	arg.dest = buffer_polytest.buffer;

	*list = (polytest_t*)_malloc_align(sizeof(polytest_t) * numleaves, 4);

	int asize ALIGNED(16);

	arg.listsize = &asize;
	arg.list = *list;
	arg.count = count;

	// Run
	SPEProgram spe_polypartition(&polypartition);
	spe_polypartition.Run(GetSPEContext(tid), &arg);

	*size = asize;

	// cout << "spu size " << (*size) << endl;

	// _free_align(leafpolys);
	
	// _free_align(count);


	buffer_polytest.Increment(buffer_polypartition.NumElements());
	buffer_polypartition.Clear();
}
#endif

void RayTraceJob::ResetPolyPartition()
{
	int numleaves = KDGetMaxNumLeaves();
	
	if( numleaves != count_size)
	{
		if(count_size > 0) 
		#ifdef _CELL
			_free_align(count);
		#else
			free(count);
		#endif
	
		count_size = numleaves;

		#ifdef _CELL
			count = (count_t*)_malloc_align(sizeof(count_t) * numleaves, 4);
		#else
			count = (count_t*)malloc(sizeof(count_t) * numleaves);
		#endif	
	}

	int i;

	for(i=0; i < numleaves; i++)
	{
		count[i].value = 0;
		count[i].offset = 0;
		count[i].ptr = 0;
	}

	/*
	int numleaves = KDGetNumLeaves();
	
	#ifdef _CELL
		count = (count_t*)_malloc_align(sizeof(count_t) * numleaves, 4);
	#else
		count = (count_t*)malloc(sizeof(count_t) * numleaves);
	#endif	

	int i;

	for(i=0; i < numleaves; i++)
	{
		count[i].value = 0;
		count[i].offset = 0;
		count[i].ptr = 0;
	}
	*/
}

void RayTraceJob::PolyPartitionCount(raystate_t *ray)
{
	count[ray->leaf].value++;
}

void RayTraceJob::PolyPartition(polytest_t **list, int *size)
{
	int numleaves = KDGetNumLeaves();
	
	raystate_t *src = buffer_polypartition.buffer;
	raystate_t *dest = buffer_polytest.buffer;

	int numelems = buffer_polypartition.NumElements();

	*size = 0;

	if(count[0].value > 0) *size = 1;

	int i;

	// Compute offsets and pointers
	for(i=1; i < numleaves; i++)
	{
		if(count[i].value > 0) (*size)++;

		count[i].offset = count[i-1].offset + count[i-1].value;
		count[i].ptr = count[i].offset;
	}

	// Move rays from polypartition to polytest
	for(i=0; i < numelems; i++)
	{
		int leaf = src[i].leaf;

		memcpy(&dest[ count[leaf].ptr++ ], &src[i], sizeof(raystate_t));
	}

	// cout << "cpu size " << (*size) << endl;

	#ifdef _CELL
		*list = (polytest_t*)_malloc_align(sizeof(polytest_t) * (*size), 4);
	#else
		*list = (polytest_t*)malloc(sizeof(polytest_t) * (*size));
	#endif
	
	int j = 0;

	for(i=0; i < numleaves; i++)
	{
		if(count[i].value > 0)
		{
			polytest_t *pp = &(*list)[j];

			pp->rays = &dest[count[i].offset];
			pp->numrays = count[i].value;
		
			KDGetLeafPolys(i, &pp->polys, &pp->numpolys);	

			j++;
		}
	}


	/*
	#ifdef _CELL
		_free_align(count);
	#else
		free(count);
	#endif
	*/

	buffer_polytest.Increment(buffer_polypartition.NumElements());
	buffer_polypartition.Clear();
}

bool Intersect(kdpoly_t *poly, raystate_t *ray)
{
	float e1[4], e2[4], s1[4];

	VectorSub(poly->triangle.v2, poly->triangle.v1, e1);
	VectorSub(poly->triangle.v3, poly->triangle.v1, e2);

	VectorCross(ray->ray.d, e2, s1);

	float divisor = VectorDot(s1, e1);

	if(divisor == 0.0f) return false;

	// First barycentric coordinate: b1
	float invDivisor = 1.0f / divisor;
	float d[4];

	VectorSub(ray->ray.o, poly->triangle.v1, d);

	float b1 = VectorDot(d, s1) * invDivisor;

	if(b1 < 0.0f || b1 > 1.0f) return false;

	// Second barycentric coordindate: b2
	float s2[4];

	VectorCross(d, e1, s2);

	float b2 = VectorDot(ray->ray.d, s2) * invDivisor;


	if(b2 < 0.0f || b1 + b2 > 1.0f) return false;

	// Compute t
	float t = VectorDot(e2, s2) * invDivisor;

	//cout << "t " << t << endl;

	if(t <= ray->ray.tmin || t >= ray->ray.tmax) return false;

	if(ray->mode != RAY_MODE_SHADOW)
	{
		ray->ray.tmax = t;
		ray->hitp[0] = ray->ray.o[0] + (ray->ray.d[0] * t);
		ray->hitp[1] = ray->ray.o[1] + (ray->ray.d[1] * t);
		ray->hitp[2] = ray->ray.o[2] + (ray->ray.d[2] * t);

		// Compute Normal
		float c = (1-b1-b2);

		ray->normal[0] = poly->normal.n1[0]*c + poly->normal.n2[0]*b1 + poly->normal.n3[0]*b2;
		ray->normal[1] = poly->normal.n1[1]*c + poly->normal.n2[1]*b1 + poly->normal.n3[1]*b2;
		ray->normal[2] = poly->normal.n1[2]*c + poly->normal.n2[2]*b1 + poly->normal.n3[2]*b2;
	}

	return true;
}


#ifdef _CELL
bool IntersectSIMD(kdpoly_t *poly, raystate_t *ray)
{
	vector float *vp0 = (vector float*)poly->triangle.v1;
	vector float *vp1 = (vector float*)poly->triangle.v2;
	vector float *vp2 = (vector float*)poly->triangle.v3;
	vector float *vray_o = (vector float*)ray->ray.o;
	vector float *vray_d = (vector float*)ray->ray.d;

	vector float e1, e2, s1;

	e1 = spu_sub(*vp1, *vp0);
	e2 = spu_sub(*vp2, *vp0);

	s1 = _cross_product3(*vray_d, e2);

	float divisor = _dot_product3(s1, e1);
		
	if(divisor == 0.0f) return false;

	float invDivisor = 1.0f / divisor;
	
	// First barycentric coordinate: b1
	vector float d = spu_sub(*vray_o, *vp0);

	float b1 = _dot_product3(d, s1) * invDivisor;

	if(b1 < 0.0f || b1 > 1.0f) return false;

	// Second barycentric coordindate: b2
	vector float s2 = _cross_product3(d, e1);

	float b2 = _dot_product3(*vray_d, s2) * invDivisor;

	if(b2 < 0.0f || b1 + b2 > 1.0f) return false;

	// Compute t
	float t = _dot_product3(e2, s2) * invDivisor;

	if(t <= ray->ray.tmin || t >= ray->ray.tmax) return false;

	ray->ray.tmax = t;
	
	vector float vt = spu_splats(t);
	vector float *vhitp = (vector float*)ray->hitp;
	*vhitp = spu_madd(*vray_d, vt, *vray_o);

	// Compute normal
	vector float *vn1 = (vector float*)poly->normal.n1;
	vector float *vn2 = (vector float*)poly->normal.n2;
	vector float *vn3 = (vector float*)poly->normal.n3;
	vector float *vnormal = (vector float*)ray->normal;

	float c = (1-b1-b2);
	vector float vc = spu_splats(c);
	vector float vb1 = spu_splats(b1);
	vector float vb2 = spu_splats(b2);

	vector float vt1 = spu_mul(*vn1, vc);
	vector float vt2 = spu_madd(*vn2, vb1, vt1);
	*vnormal = spu_madd(*vn3, vb2, vt2);

	return true;
}
#endif 

void PhongColor(float lightpos[3], float lightdiffuse[3], float hitp[3], float normal[3], float color[3], float ret[3])
{
	float diffuse[3] = {0,0,0};
	float L[4];

	VectorSub(lightpos, hitp, L);
	VectorNormalize(L);

	float dotLN = VectorDot(L, normal);

	if(dotLN > 0)
	{
		diffuse[0] = color[0] * lightdiffuse[0] * dotLN;
		diffuse[1] = color[1] * lightdiffuse[1] * dotLN;
		diffuse[2] = color[2] * lightdiffuse[2] * dotLN;
	}

	ret[0] = diffuse[0];
	ret[1] = diffuse[1];
	ret[2] = diffuse[2];
}


#ifdef _CELL
void PhongColorSIMD(float lightpos[3], float lightdiffuse[3], float hitp[3], float normal[3], float color[3], float ret[3])
{
	float p[4] ALIGNED(16) = { hitp[0], hitp[1], hitp[2], 0};
	float n[4] ALIGNED(16) = { normal[0], normal[1], normal[2], 0};
	float c[4] ALIGNED(16) = { color[0], color[1], color[2], 0};
	float lpos[4] ALIGNED(16) = { lightpos[0], lightpos[1], lightpos[2], 0};
	float ldiffuse[4] ALIGNED(16) = { lightdiffuse[0], lightdiffuse[1], lightdiffuse[2], 0};

	vector float *vp = (vector float*)p;
	vector float *vlpos = (vector float*)lpos;
	vector float *vldiffuse = (vector float*)ldiffuse;
	vector float *vn = (vector float*)n;
	vector float *vc = (vector float*)c;

	vector float L = _normalize3( spu_sub(*vlpos, *vp) );
	float dotLN = _dot_product3(L, *vn);

	float diffuse[4] ALIGNED(16) = { 0, 0, 0, 0};

	vector float* vdiffuse = (vector float*)diffuse;

	if(dotLN > 0)
	{	
		vector float vdotLN = spu_splats(dotLN);	
		*vdiffuse = spu_mul(vdotLN, spu_mul(*vc, *vldiffuse));
	}	

	ret[0] = diffuse[0];
	ret[1] = diffuse[1];
	ret[2] = diffuse[2];
}
#endif

#ifdef _CELL
void RayTraceJob::PolyTestSPU(polytest_t *list, int size, float lightpos[3], float lightdiffuse[3])
{
	polytest_arg_t arg ALIGNED(16);
	int numhits ALIGNED(16);
	int a_npolytests ALIGNED(16);	

	a_npolytests = 0;
	numhits = 0;

	memcpy(arg.lightpos, lightpos, sizeof(float[3]));
	memcpy(arg.lightdiffuse, lightdiffuse, sizeof(float[3]));

	int numrays = 0;

	for(int i=0; i < size; i++) 
	{	
		numrays += list[i].numrays;
		npolytests += (list[i].numrays * list[i].numpolys);
	}

	if(GetScene()->polytest_kernel == K_POLYTEST_SIMD_SPU)
		arg.simd = 1;
	else
		arg.simd = 0;

	arg.polytests = list;
	arg.numpolytests = size;
	arg.buffer_kdtravers = (raystate_t*)buffer_kdtravers.CopyToPtr();
	arg.buffer_raster = (raystate_t*)buffer_raster.CopyToPtr();
	arg.numhits = &numhits;
	arg.npolytests = &a_npolytests;
	
	arg.shadowrays = GetScene()->shadowrays;
	arg.secondrays = GetScene()->secondrays;
	arg.shading = GetScene()->shading;

	// Run!
	SPEProgram spe_polytest(&polytest);
	spe_polytest.Run(GetSPEContext(tid), &arg);
	
	buffer_raster.Increment(numhits);
	buffer_kdtravers.Increment(numrays - numhits);			

	_free_align(list);	
	buffer_polytest.Clear();

	//npolytests += a_npolytests;
}
#endif

#ifdef _CELL
void ShadowRayGenSIMD(raystate_t *ray, float lpos[4])
{
	ray->mode = RAY_MODE_SHADOW;
	ray->ep = 0;
	ray->leaf = -1;
	ray->leafcount = 0;
	
	vector float *ray_o = (vector float*)ray->ray.o;
	vector float *ray_d = (vector float*)ray->ray.d;
	vector float *ray_n = (vector float*)ray->normal;
	vector float *hitp = (vector float*)ray->hitp;
	vector float *vlpos = (vector float*)lpos;
	
	*ray_d = spu_sub(*hitp, *vlpos);
	float dist = _length_vec3(*ray_d);
	*ray_d = _normalize3(*ray_d);
	
	*ray_o = *vlpos;
			
	ray->ray.tmin = 0.0001f;
	ray->ray.tmax = dist - 0.0001f;
}


void ReflectiveRayGenSIMD(raystate_t *ray)
{
	ray->mode = RAY_MODE_REFLECTION;
	ray->ep = 0;
	ray->leaf = -1;
	ray->leafcount = 0;
	ray->depth += 1;

	vector float *ray_o = (vector float*)ray->ray.o;
	vector float *ray_d = (vector float*)ray->ray.d;
	vector float *ray_n = (vector float*)ray->normal;
	vector float *hitp = (vector float*)ray->hitp;
	vector float *org_dir = (vector float*)ray->org_dir;

	float dotDN2 = 2.0f * _dot_product3(*org_dir, *ray_n);
	vector float NdotDN2 = spu_mul(*ray_n, spu_splats(dotDN2));
	*ray_d = spu_sub(*org_dir, NdotDN2);
	*ray_o = *hitp;
	
	ray->ray.tmin = 0.001f;
	ray->ray.tmax = 1000000.0f;
}


#endif


void ShadowRayGen(raystate_t *ray, float lpos[3])
{
	ray->mode = RAY_MODE_SHADOW;
	ray->ep = 0;
	ray->leaf = -1;
	ray->leafcount = 0;
	
	VectorSub(ray->hitp, lpos, ray->ray.d);

	float dist = fabs(VectorLen(ray->ray.d));

	VectorNormalize(ray->ray.d);

	memcpy(ray->ray.o, lpos, sizeof(float[3]));
						
	ray->ray.tmin = 0.0001f;
	ray->ray.tmax = dist - 0.0001f;
}

void ReflectiveRayGen(raystate_t *ray)
{
	ray->mode = RAY_MODE_REFLECTION;
	ray->ep = 0;
	ray->leaf = -1;
	ray->leafcount = 0;
	ray->depth += 1;
	
	float R[4];
	float NdotDN2[4];

	// R = V - 2 * (N . V) * N

	float dotDN2 = 2.0f * VectorDot(ray->org_dir, ray->normal);
	VectorScale(ray->normal, dotDN2, NdotDN2);
	VectorSub(ray->org_dir, NdotDN2, ray->ray.d);

	memcpy(ray->ray.o, ray->hitp, sizeof(float[4]));

	ray->ray.tmin = 0.001f;
	ray->ray.tmax = 1000000.0f;
}

int PolyTestSort(raystate_t *rays, int numrays)
{
	int c = 0;	
	int r;


	if(GetScene()->secondrays)
	{
		for(r=c; r < numrays; r++)
		{
			if(rays[r].mode == RAY_MODE_REFLECTION && (rays[r].phit & RAY_REFLECTIVE_HIT || rays[r].phit & RAY_REFLECTIVE_MISS) )
			{	
				if(!(rays[r].phit & RAY_REFLECTIVE_BIT))	
				{
					float re = rays[r].color.reflective;
						
					rays[r].color.c[0] = (rays[r].color.c[0] * (1-re)) + (rays[r].rcolor.c[0] * re);
					rays[r].color.c[1] = (rays[r].color.c[1] * (1-re)) + (rays[r].rcolor.c[1] * re);
					rays[r].color.c[2] = (rays[r].color.c[2] * (1-re)) + (rays[r].rcolor.c[2] * re);
				}

				MemorySwap(&rays[c], &rays[r], sizeof(raystate_t));
				c++;
			}
		
		}

	}


	if(GetScene()->shadowrays)
	{
		for(r=c; r < numrays; r++)
		{
			if(rays[r].mode == RAY_MODE_SHADOW && (rays[r].phit & RAY_SHADOW_HIT || rays[r].phit & RAY_SHADOW_MISS) )
			{	
				MemorySwap(&rays[c], &rays[r], sizeof(raystate_t));
				c++;
			}
		}
	}

	
	for(r=c; r < numrays; r++)
	{
		if(rays[r].mode == RAY_MODE_PRIMARY && (rays[r].phit & RAY_PRIMARY_HIT) && !(rays[r].phit & RAY_REFLECTIVE_BIT))
		{	
			MemorySwap(&rays[c], &rays[r], sizeof(raystate_t));
			c++;
		}
	}


	for(r=c; r < numrays; r++)
	{
		if(rays[r].mode == RAY_MODE_PRIMARY && (rays[r].phit & RAY_PRIMARY_HIT) && (rays[r].phit & RAY_REFLECTIVE_BIT) )
		{	
			MemorySwap(&rays[c], &rays[r], sizeof(raystate_t));
			c++;
		}

	}

	return c;
}

int RayGenSecondary(raystate_t *rays, int numrays, float lightpos[4])
{
	int r;
	int c = 0;

	if(GetScene()->shadowrays || GetScene()->secondrays)
	{

		for(r=0; r < numrays; r++)
		{
			if((rays[r].mode == RAY_MODE_PRIMARY) && (rays[r].phit & RAY_PRIMARY_HIT))
			{
				if(GetScene()->shadowrays)
				{
					memcpy(rays[r].org_dir, rays[r].ray.d, sizeof(float[3]));
					ShadowRayGen(&rays[r], lightpos);
					c++;
				}
				else
				{
					if(GetScene()->secondrays)
					{
						if(rays[r].phit & RAY_REFLECTIVE_BIT)
						{
							memcpy(rays[r].org_dir, rays[r].ray.d, sizeof(float[3]));
							ReflectiveRayGen(&rays[r]);
							c++;
						}
					}

				}
			}


			if(GetScene()->secondrays)
			{
				if((rays[r].mode == RAY_MODE_SHADOW) && ( rays[r].phit & RAY_SHADOW_HIT || rays[r].phit & RAY_SHADOW_MISS))
				{
					if(rays[r].phit & RAY_REFLECTIVE_BIT)
					{
						ReflectiveRayGen(&rays[r]);
						c++;
					}

				}
			}

		}
	}


	return c;
}





void RayTraceJob::PolyTest(polytest_t *list, int size, float lpos[3], float ldiffuse[3])
{
	float lightpos[4] ALIGNED(16) = { lpos[0], lpos[1], lpos[2], 0};
	float lightdiffuse[4] ALIGNED(16) = { ldiffuse[0], ldiffuse[1], ldiffuse[2], 0};
	
	for(int i=0; i < size; i++)
	{
		int hitcount = 0;
	
		polytest_t pp = list[i];

		raystate_t *rays = pp.rays; 
		int numrays = pp.numrays;

		for(int r=0; r < numrays; r++)
		{
			if(rays[r].mode == RAY_MODE_SHADOW && rays[r].phit & RAY_SHADOW_MISS)
			{
				continue;
			}

			if(rays[r].mode == RAY_MODE_REFLECTION && rays[r].phit & RAY_REFLECTIVE_MISS)
			{
				continue;
			}

			int polyhit = -1;

			int run = 1;

			npolytests += pp.numpolys;	

			for(int p=0; p < pp.numpolys && run; p++)
			{
				bool intersect;

				#ifdef _CELL
				if(GetScene()->polytest_kernel == K_POLYTEST_SIMD)
					intersect = IntersectSIMD(&pp.polys[p], &rays[r]);
				else
				#endif
					intersect = Intersect(&pp.polys[p], &rays[r]);

				if(intersect)
				{
					rays[r].phit |= 1;
					polyhit = p;

					if(rays[r].mode == RAY_MODE_SHADOW) run = 0;
				}
			}
		
			if(polyhit >= 0)
			{

				if(rays[r].mode == RAY_MODE_PRIMARY)
				{				
					if(lpos != 0 && GetScene()->shading)
					{
						#ifdef _CELL
						if(GetScene()->polytest_kernel == K_POLYTEST_SIMD)
							PhongColorSIMD(lightpos, lightdiffuse, rays[r].hitp, rays[r].normal, pp.polys[polyhit].color, rays[r].color.c);
						else
						#endif
							PhongColor(lightpos, lightdiffuse, rays[r].hitp, rays[r].normal, pp.polys[polyhit].color, rays[r].color.c);
					}
					else
					{
						rays[r].color.c[0] = pp.polys[polyhit].color[0];
						rays[r].color.c[1] = pp.polys[polyhit].color[1];
						rays[r].color.c[2] = pp.polys[polyhit].color[2];
					}

					if(pp.polys[polyhit].reflective > 0) 
					{
						rays[r].phit |= RAY_REFLECTIVE_BIT;
						rays[r].color.reflective = pp.polys[polyhit].reflective;
					}
					else
					{
						rays[r].phit &= ~RAY_REFLECTIVE_BIT;
					}

					rays[r].phit |= RAY_PRIMARY_HIT;	
				}


				if(rays[r].mode == RAY_MODE_SHADOW)
				{
					rays[r].phit |= RAY_SHADOW_HIT;
				}

				if(rays[r].mode == RAY_MODE_REFLECTION)
				{
					rays[r].phit |= RAY_REFLECTIVE_HIT;

					if(pp.polys[polyhit].reflective == 0)
					{
						float c[4] ALIGNED(16);

						#ifdef _CELL
						if(GetScene()->polytest_kernel == K_POLYTEST_SIMD)
							PhongColorSIMD(lightpos, lightdiffuse, rays[r].hitp, rays[r].normal, pp.polys[polyhit].color, rays[r].rcolor.c);
						else
						#endif
							PhongColor(lightpos, lightdiffuse, rays[r].hitp, rays[r].normal, pp.polys[polyhit].color, rays[r].rcolor.c);

						rays[r].phit &= ~RAY_REFLECTIVE_BIT;
					}
					else
					{
						rays[r].phit |= RAY_REFLECTIVE_BIT;
						rays[r].color.reflective = pp.polys[polyhit].reflective;
					}
				}

			}
		}

		hitcount = PolyTestSort(rays, numrays);
		hitcount -= RayGenSecondary(rays, numrays, lpos);


		buffer_raster.CopyTo(rays, hitcount);
		buffer_kdtravers.CopyTo(&rays[hitcount], numrays - hitcount);
	}

	#ifdef _CELL
		_free_align(list);
	#else
		free(list);
	#endif

	buffer_polytest.Clear();
}
