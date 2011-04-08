#include <stdio.h>
#include <polytest.h>
#include <spu_intrinsics.h>
#include <spu_mfcio.h>
#include <cross_product3.h>
#include <dot_product3.h>
#include <normalize3.h>
#include <length_vec3.h>
#include <string.h>
#include <math.h>
#include <dma.h>
#include <doublebuf.h>
#include <triplebuf.h>

typedef unsigned int uint;

#define BUFFERSIZE		32*1024

#define NUMRAYS			(4*1024)/sizeof(raystate_t)
#define NUMPOLYS		(4*1024)/sizeof(kdpoly_t)
#define NUMPOLYTESTS	BUFFERSIZE/sizeof(polytest_t)

volatile polytest_arg_t arg;

raystate_t raybuffer[3][ NUMRAYS ] ALIGNED(16);
kdpoly_t polybuffer[2][ NUMPOLYS ] ALIGNED(16);
polytest_t polytestbuffer[ NUMPOLYTESTS ] ALIGNED(16);

triplebuf_t ray_tb;
doublebuf_t poly_db;

struct mfc_list_element polytestlist[16] ALIGNED(16);
struct mfc_list_element kdtraverslist[3][16] ALIGNED(16);
struct mfc_list_element rasterlist[3][16] ALIGNED(16);

int numhits ALIGNED(16);

int shading = 1;
int shadowrays = 0;
int secondrays = 1;
int maxraydepth = 3;
int npolytests ALIGNED(16);

float clear_color[3] = { 0.2f, 0.2f, 0.3f };

inline float VectorLen(float *v)
{
	return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

inline void VectorScale(float *v, float s, float *ret)
{
	ret[0] = v[0]*s;
	ret[1] = v[1]*s;
	ret[2] = v[2]*s;
}


void VectorSub(float *v1, float *v2, float *ret)
{
	ret[0] = v1[0] - v2[0]; 
	ret[1] = v1[1] - v2[1]; 
	ret[2] = v1[2] - v2[2]; 
}

void VectorCross(float *v1, float *v2, float *ret)
{
	ret[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
	ret[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
	ret[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

float VectorDot(float *v1, float *v2)
{
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] *v2[2];
}

void VectorNormalize(float *v)
{
	float len = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
	float invlen = 1/len;

	v[0] *= invlen;
	v[1] *= invlen;
	v[2] *= invlen;
}


int Intersect(kdpoly_t *poly, raystate_t *ray)
{
	float e1[4], e2[4], s1[4];

	VectorSub(poly->triangle.v2, poly->triangle.v1, e1);
	VectorSub(poly->triangle.v3, poly->triangle.v1, e2);

	VectorCross(ray->ray.d, e2, s1);

	float divisor = VectorDot(s1, e1);

	if(divisor == 0.0f) return 0;

	// First barycentric coordinate: b1
	float invDivisor = 1.0f / divisor;
	float d[4];

	VectorSub(ray->ray.o, poly->triangle.v1, d);

	float b1 = VectorDot(d, s1) * invDivisor;

	if(b1 < 0.0f || b1 > 1.0f) return 0;

	// Second barycentric coordindate: b2
	float s2[4];

	VectorCross(d, e1, s2);

	float b2 = VectorDot(ray->ray.d, s2) * invDivisor;

	if(b2 < 0.0f || b1 + b2 > 1.0f) return 0;

	// Compute t
	float t = VectorDot(e2, s2) * invDivisor;

	if(t <= ray->ray.tmin || t >= ray->ray.tmax) return 0;

	ray->ray.tmax = t;
	ray->hitp[0] = ray->ray.o[0] + (ray->ray.d[0] * t);
	ray->hitp[1] = ray->ray.o[1] + (ray->ray.d[1] * t);
	ray->hitp[2] = ray->ray.o[2] + (ray->ray.d[2] * t);

	// Compute Normal
	float c = (1-b1-b2);

	ray->normal[0] = poly->normal.n1[0]*c + poly->normal.n2[0]*b1 + poly->normal.n3[0]*b2;
	ray->normal[1] = poly->normal.n1[1]*c + poly->normal.n2[1]*b1 + poly->normal.n3[1]*b2;
	ray->normal[2] = poly->normal.n1[2]*c + poly->normal.n2[2]*b1 + poly->normal.n3[2]*b2;

	return 1;
}


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


inline int IntersectSIMD(kdpoly_t *poly, raystate_t *ray)
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
		
	if(divisor == 0.0f) return 0;

	float invDivisor = 1.0f / divisor;
	
	// First barycentric coordinate: b1
	vector float d = spu_sub(*vray_o, *vp0);

	float b1 = _dot_product3(d, s1) * invDivisor;

	if(b1 < 0.0f || b1 > 1.0f) return 0;

	// Second barycentric coordindate: b2
	vector float s2 = _cross_product3(d, e1);

	float b2 = _dot_product3(*vray_d, s2) * invDivisor;

	if(b2 < 0.0f || b1 + b2 > 1.0f) return 0;

	// Compute t
	float t = _dot_product3(e2, s2) * invDivisor;

	if(t <= ray->ray.tmin || t >= ray->ray.tmax) return 0;

	if(ray->mode != RAY_MODE_SHADOW)
	{
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
	}

	return 1;
}

void PhongColorSIMD(float lightpos[3], float lightdiffuse[3], float hitp[3], float normal[3], float color[3], float ret[3])
{
	vector float *vp = (vector float*)hitp;
	vector float *vlpos = (vector float*)lightpos;
	vector float *vldiffuse = (vector float*)lightdiffuse;
	vector float *vn = (vector float*)normal;
	vector float *vc = (vector float*)color;
	vector float *vret = (vector float*)ret;

	vector float L = _normalize3( spu_sub(*vlpos, *vp) );
	float dotLN = _dot_product3(L, *vn);

	float f = 0;
	*vret = spu_splats(f); // (vector float*)diffuse;

	if(dotLN > 0)
	{	
		vector float vdotLN = spu_splats(dotLN);	
		*vret = spu_mul(vdotLN, spu_mul(*vc, *vldiffuse));
	}	
}

inline void MemorySwap(raystate_t *e1, raystate_t *e2)
{
	if(e1 == e2) return;

	raystate_t tmp;	

	memcpy(&tmp, e1, sizeof(raystate_t));
	memcpy(e1, e2, sizeof(raystate_t));
	memcpy(e2, &tmp, sizeof(raystate_t));
}


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
	
	float NdotDN2[4];

	// R = V - 2 * (N . V) * N

	float dotDN2 = 2.0f * VectorDot(ray->org_dir, ray->normal);
	VectorScale(ray->normal, dotDN2, NdotDN2);
	VectorSub(ray->org_dir, NdotDN2, ray->ray.d);

	memcpy(ray->ray.o, ray->hitp, sizeof(float[4]));

	ray->ray.tmin = 0.001f;
	ray->ray.tmax = 1000000.0f;
}


void ShadowRayGenSIMD(raystate_t *ray, float lpos[4])
{
	ray->mode = RAY_MODE_SHADOW;
	ray->ep = 0;
	ray->leaf = -1;
	ray->leafcount = 0;
	
	vector float *ray_o = (vector float*)ray->ray.o;
	vector float *ray_d = (vector float*)ray->ray.d;
	vector float *hitp = (vector float*)ray->hitp;
	vector float *vlpos = (vector float*)lpos;
	
	*ray_d = spu_sub(*hitp, *vlpos);
	float dist = _length_vec3(*ray_d);
	*ray_d = _normalize3(*ray_d);
	
	*ray_o = *vlpos;
			
	ray->ray.tmin = 0.001f;
	ray->ray.tmax = dist - 0.001f;
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

int PolyTestSort(raystate_t *rays, int numrays)
{
	int c = 0;	
	int r;


	if(secondrays)
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

				MemorySwap(&rays[c], &rays[r]);
				c++;
			}
		
		}

	}


	if(shadowrays)
	{
		for(r=c; r < numrays; r++)
		{
			if(rays[r].mode == RAY_MODE_SHADOW && (rays[r].phit & RAY_SHADOW_HIT || rays[r].phit & RAY_SHADOW_MISS) )
			{	
				MemorySwap(&rays[c], &rays[r]);
				c++;
			}
		}
	}

	
	for(r=c; r < numrays; r++)
	{
		if(rays[r].mode == RAY_MODE_PRIMARY && (rays[r].phit & RAY_PRIMARY_HIT) && !(rays[r].phit & RAY_REFLECTIVE_BIT))
		{	
			MemorySwap(&rays[c], &rays[r]);
			c++;
		}
	}


	for(r=c; r < numrays; r++)
	{
		if(rays[r].mode == RAY_MODE_PRIMARY && (rays[r].phit & RAY_PRIMARY_HIT) && (rays[r].phit & RAY_REFLECTIVE_BIT) )
		{	
			MemorySwap(&rays[c], &rays[r]);
			c++;
		}

	}

	return c;
}

int RayGenSecondary(raystate_t *rays, int numrays, float lightpos[4])
{
	int r;
	int c = 0;

	if(shadowrays || secondrays)
	{

		for(r=0; r < numrays; r++)
		{
			if((rays[r].mode == RAY_MODE_PRIMARY) && (rays[r].phit & RAY_PRIMARY_HIT))
			{
				if(shadowrays)
				{
					memcpy(rays[r].org_dir, rays[r].ray.d, sizeof(float[3]));

					if(arg.simd)
						ShadowRayGenSIMD(&rays[r], lightpos);
					else
						ShadowRayGen(&rays[r], lightpos);

					c++;
				}
				else
				{
					if(secondrays)
					{
						if(rays[r].phit & RAY_REFLECTIVE_BIT)
						{
							memcpy(rays[r].org_dir, rays[r].ray.d, sizeof(float[3]));

							if(arg.simd)
								ReflectiveRayGenSIMD(&rays[r]);
							else
								ReflectiveRayGen(&rays[r]);
							c++;
						}
					}

				}
			}


			if(secondrays)
			{
				if((rays[r].mode == RAY_MODE_SHADOW) && ( rays[r].phit & RAY_SHADOW_HIT || rays[r].phit & RAY_SHADOW_MISS))
				{
					if(rays[r].phit & RAY_REFLECTIVE_BIT)
					{
						if(arg.simd)
							ReflectiveRayGenSIMD(&rays[r]);
						else
							ReflectiveRayGen(&rays[r]);
						c++;
					}

				}
			}

		}
	}


	return c;
}


void PolyTest(raystate_t *rays, int numrays, kdpoly_t *polys, int numpolys, float lightpos[3], float lightdiffuse[3])
{
	if(numpolys <= 0) return;

	int r,p;

	for(r=0; r < numrays; r++)
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

		for(p=0; p < numpolys && run; p++)
		{
			int intersect;

			if(arg.simd)
				intersect = IntersectSIMD(&polys[p], &rays[r]);
			else
				intersect = Intersect(&polys[p], &rays[r]);


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
				if(lightpos != 0 && shading)
				{
					if(arg.simd)
						PhongColorSIMD(lightpos, lightdiffuse, rays[r].hitp, rays[r].normal, polys[polyhit].color, rays[r].color.c);
					else
						PhongColor(lightpos, lightdiffuse, rays[r].hitp, rays[r].normal, polys[polyhit].color, rays[r].color.c);
				}
				else
				{
					rays[r].color.c[0] = polys[polyhit].color[0];
					rays[r].color.c[1] = polys[polyhit].color[1];
					rays[r].color.c[2] = polys[polyhit].color[2];
				}

				if(polys[polyhit].reflective > 0) 
				{
					rays[r].phit |= RAY_REFLECTIVE_BIT;
					rays[r].color.reflective = polys[polyhit].reflective;
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

				if(polys[polyhit].reflective == 0)
				{
					if(arg.simd)
						PhongColorSIMD(lightpos, lightdiffuse, rays[r].hitp, rays[r].normal, polys[polyhit].color, rays[r].rcolor.c);
					else
						PhongColor(lightpos, lightdiffuse, rays[r].hitp, rays[r].normal, polys[polyhit].color, rays[r].rcolor.c);

					rays[r].phit &= ~RAY_REFLECTIVE_BIT;
				}
				else
				{
					rays[r].phit |= RAY_REFLECTIVE_BIT;
					rays[r].color.reflective =polys[polyhit].reflective;
				}
			}

		}		
	}
}

inline void Decrement(int *total, int max, int *value)
{
	if(*total <= 0)
	{
		*value = 0;
		return;
	}

	if(*total > max)
	{
		*value = max;
		*total -= max;
	}
	else
	{
		*value = *total;
		*total = 0;	
	}
}

void PolyTestAll(kdpoly_t *polys, int numpolys, int ridx, int ray_size)
{
	npolytests += (ray_size * numpolys);	

	DmaWaitAll();

	DoubleBufResetEA(&poly_db, numpolys, polys);	

	numhits = 0;

	int poly_size[2];

	DoubleBufWait(&poly_db, 0);
	DoubleBufWait(&poly_db, 1);

	poly_size[0] = DoubleBufGet(&poly_db, 0);
	poly_size[1] = DoubleBufGet(&poly_db, 1);

	while(!DoubleBufEmpty(&poly_db))
	{
		DoubleBufWait(&poly_db, 0);
		PolyTest(raybuffer[ridx], ray_size, polybuffer[0], poly_size[0], arg.lightpos, arg.lightdiffuse);		
		poly_size[0] = DoubleBufGet(&poly_db, 0);

		DoubleBufWait(&poly_db, 1);
		PolyTest(raybuffer[ridx], ray_size, polybuffer[1], poly_size[1], arg.lightpos, arg.lightdiffuse);		
		poly_size[1] = DoubleBufGet(&poly_db, 1);			
	}			

	DoubleBufWait(&poly_db, 0);
	PolyTest(raybuffer[ridx], ray_size, polybuffer[0], poly_size[0], arg.lightpos, arg.lightdiffuse);

	DoubleBufWait(&poly_db, 1);
	PolyTest(raybuffer[ridx], ray_size, polybuffer[1], poly_size[1], arg.lightpos, arg.lightdiffuse);

	numhits = PolyTestSort(raybuffer[ridx], ray_size);
	numhits -= RayGenSecondary(raybuffer[ridx], ray_size, arg.lightpos);
}


int main(unsigned long long spu_id __attribute__ ((unused)), unsigned long long parm)
{
	uint kdtravers_id[3];
	uint raster_id[3];	
	uint tag_id;

	tag_id = mfc_tag_reserve();
	kdtravers_id[0] = mfc_tag_reserve();
	kdtravers_id[1] = mfc_tag_reserve();
	kdtravers_id[2] = mfc_tag_reserve();
	raster_id[0] = mfc_tag_reserve();
	raster_id[1] = mfc_tag_reserve();
	raster_id[2] = mfc_tag_reserve();


	// Transfer arg
	spu_mfcdma32(&arg, parm, sizeof(polytest_arg_t), tag_id, MFC_GET_CMD);
	DmaWait(tag_id);	
	
	shadowrays = arg.shadowrays;
	secondrays = arg.secondrays;
	shading = arg.shading;
	
	npolytests = 0;

	int polytests_size = 0;	
	int polytests_offset = 0;

	int numpolytests = arg.numpolytests;
	int offset_raster = 0;
	int offset_kdtravers = 0;

	int totalnumhits = 0;
	
	TripleBufInit(&ray_tb, 0, 0, sizeof(raystate_t), NUMRAYS, raybuffer[0], raybuffer[1], raybuffer[2]);
	DoubleBufInit(&poly_db, 0, 0, sizeof(kdpoly_t), NUMPOLYS, polybuffer[0], polybuffer[1]);


	do	
	{
		// Transfer polytest structs
		Decrement(&numpolytests, NUMPOLYTESTS, &polytests_size);
		DmaGet(polytestlist, polytestbuffer, (uint)&arg.polytests[polytests_offset], polytests_size * sizeof(polytest_t), tag_id);
		polytests_offset += polytests_size;
		DmaWait(tag_id);
	
		int ray_size[3];
		int i;
		int nummiss;

		ray_size[0] = 0;
		ray_size[1] = 0;
		ray_size[2] = 0;

		// printf("polytests_size %i\n", polytests_size);

		for(i=0; i < polytests_size; i++)
		{
			polytest_t *pt = &polytestbuffer[i];

			TripleBufWait(&ray_tb, 0);		
			DmaWait(raster_id[0]);
			DmaWait(kdtravers_id[0]);

			TripleBufResetEA(&ray_tb, pt->numrays, pt->rays);

			ray_size[0] = TripleBufGet(&ray_tb, 0);	
		
			while(!TripleBufEmpty(&ray_tb))
			{
				// Transfer to buffer 2										
				DmaWait(raster_id[1]);
				DmaWait(kdtravers_id[1]);		
				ray_size[1] = TripleBufGet(&ray_tb, 1);

				// Buffer 1
				TripleBufWait(&ray_tb, 0);		
				DmaWait(raster_id[0]);
				DmaWait(kdtravers_id[0]);

				PolyTestAll(pt->polys, pt->numpolys, 0, ray_size[0]);				
							
				// Transfer hit rays to raster_buffer
				DmaPut(rasterlist[0], raybuffer[0], (uint)&arg.buffer_raster[offset_raster],  numhits * sizeof(raystate_t), raster_id[0]);
				offset_raster += numhits;

				// Transfer miss rays to kdtravers_buffer
				nummiss = ray_size[0] - numhits;
				DmaPut(kdtraverslist[0], &raybuffer[0][numhits], (uint)&arg.buffer_kdtravers[offset_kdtravers], nummiss * sizeof(raystate_t), kdtravers_id[0]);
				offset_kdtravers += nummiss;				
				
				ray_size[0] = 0;

				totalnumhits += numhits;
				
				// Transfer to buffer 3	
				DmaWait(raster_id[2]);
				DmaWait(kdtravers_id[2]);	
				ray_size[2] = TripleBufGet(&ray_tb, 2);
				
				// Buffer 2
				TripleBufWait(&ray_tb, 1);		
				DmaWait(raster_id[1]);
				DmaWait(kdtravers_id[1]);

				PolyTestAll(pt->polys, pt->numpolys, 1, ray_size[1]);
	
				// Transfer hit rays to raster_buffer
				DmaPut(rasterlist[1], raybuffer[1], (uint)&arg.buffer_raster[offset_raster],  numhits * sizeof(raystate_t), raster_id[1]);
				offset_raster += numhits;

				// Transfer miss rays to kdtravers_buffer
				nummiss = ray_size[1] - numhits;
				DmaPut(kdtraverslist[1], &raybuffer[1][numhits], (uint)&arg.buffer_kdtravers[offset_kdtravers], nummiss * sizeof(raystate_t), kdtravers_id[1]);
				offset_kdtravers += nummiss;				

				totalnumhits += numhits;

				// Transfer to buffer 1	
				DmaWait(raster_id[0]);
				DmaWait(kdtravers_id[0]);	

				ray_size[0] = TripleBufGet(&ray_tb, 0);	

				// Buffer 3
				TripleBufWait(&ray_tb, 2);		
				DmaWait(raster_id[2]);
				DmaWait(kdtravers_id[2]);

				PolyTestAll(pt->polys, pt->numpolys, 2, ray_size[2]);

				// Transfer hit rays to raster_buffer
				DmaPut(rasterlist[2], raybuffer[2], (uint)&arg.buffer_raster[offset_raster],  numhits * sizeof(raystate_t), raster_id[2]);
				offset_raster += numhits;

				// Transfer miss rays to kdtravers_buffer
				nummiss = ray_size[2] - numhits;
				DmaPut(kdtraverslist[2], &raybuffer[2][numhits], (uint)&arg.buffer_kdtravers[offset_kdtravers], nummiss * sizeof(raystate_t), kdtravers_id[2]);
				offset_kdtravers += nummiss;						

				totalnumhits += numhits;
			}	

			// Buffer 1
			TripleBufWait(&ray_tb, 0);		
			DmaWait(raster_id[0]);
			DmaWait(kdtravers_id[0]);

			PolyTestAll(pt->polys, pt->numpolys, 0, ray_size[0]);				
			
			// Transfer hit rays to raster_buffer
			DmaPut(rasterlist[0], raybuffer[0], (uint)&arg.buffer_raster[offset_raster],  numhits * sizeof(raystate_t), raster_id[0]);
			offset_raster += numhits;

			// Transfer miss rays to kdtravers_buffer
			nummiss = ray_size[0] - numhits;
			DmaPut(kdtraverslist[0], &raybuffer[0][numhits], (uint)&arg.buffer_kdtravers[offset_kdtravers], nummiss * sizeof(raystate_t), kdtravers_id[0]);
			offset_kdtravers += nummiss;				

			totalnumhits += numhits;

		}
	} while( numpolytests > 0);


	spu_mfcdma32(&totalnumhits, (unsigned int)arg.numhits, sizeof(int), tag_id, MFC_PUT_CMD);
	
	DmaWait(tag_id);	

	spu_mfcdma32(&npolytests, (unsigned int)arg.npolytests, sizeof(int), tag_id, MFC_PUT_CMD);

	DmaWaitAll();

	return 0;
}

