#include <list>
#include <math.h>
#include <stdio.h>
#include <mesh/aabb.h>
#include <scene/kdbuildjob.h>
#include <scene/math3d.h>
#include <scene/scene.h>
#include <share/structs.h>
#include <share/kdbuffer.h>

#ifdef _CELL
	#include <vec_types.h>
	#include <altivec.h>
	#include <spu2vmx.h>
	#include <spu/kernels/kdbuild.h>
#endif

#ifdef _CELL
	extern spe_program_handle_t kdbuild;
#endif


static int nsamplepoints = 16;
static int nsplitaxises = 3;
static kdpolyp_t *polys;
static int numpolys = 0;

static kdnode_t *nodes;
static int numnodes = 0;

static int *leaves;
static int numleaves = 0;
static int maxnumleaves;

static int numleafpolys = 0;
static int maxdepth = 0;

static aabb_t worldaabb;

kdleafpoly_t *leafpolys;

void KDPrintLeaves()
{
	for(int i=0; i < numleaves; i++)
	{
		if(leaves[i] > 0)
		{
			cout << "Leaf " << i << " node " << leaves[i] << endl;
		}
	}

}

static inline int IsLeaf(kdnode_t *node)
{
	return !node->left;
}

void PackKDTree()
{
	int moved[numnodes];

	Scene *scene = GetScene();

	KDBuildJob *job = scene->kdjob;
}


int KDGetMaxDepth()
{
	return maxdepth;
}


kdpolyp_t *KDGetPolys()
{
	return polys;
}

int KDGetNumPolys()
{
	return numpolys;
}

int KDGetMaxNumLeaves()
{
	return maxnumleaves;
}


void KDIncNumberOfSplits()
{
	if(nsamplepoints < 512)
	{
		nsamplepoints *= 2;
	}
}

void KDDecNumberOfSplits()
{
	if(nsamplepoints > 2)
	{
		nsamplepoints /= 2;
	}
}

int KDGetNumberOfAxises()
{
	return nsplitaxises;
}

void KDChangeNumberOfAxises()
{
	nsplitaxises = 1 + (nsplitaxises % 3);
}

void KDSetNumberOfSplits(int n)
{
	nsamplepoints = n;
}

void KDSetNumberOfAxises(int n)
{
	nsplitaxises = n;
}


int KDGetNumberOfSplits()
{
	return nsamplepoints;
}

int KDGetNumLeafPolys()
{
	return numleafpolys;
}

int KDGetNumNodes()
{
	return numnodes;
}

int KDGetNumLeaves()
{
	return maxnumleaves;
}

kdnode_t* KDGetNodes(int *nnodes = 0)
{
	if(nnodes != 0) *nnodes = numnodes;

	return nodes;
}

aabb_t *KDGetWorldAABB()
{
	return &worldaabb;
}

void KDGetLeafPolys(int leaf, kdpoly_t **polys, int *numpolys)
{
	if(leaf >= 0)
	{
		int n = leaves[leaf];

		*numpolys = nodes[n].numpolys;
		*polys = nodes[n].polys;	
	}
	else
	{
		*numpolys = 0;
		*polys = 0;
	}

}


void ComputeWorldAABB(aabb_t *worldaabb, Scene *scene)
{
	vector<Mesh*>::iterator it;

	AABB waabb(worldaabb);

	waabb.Reset();

	for(it = scene->meshlist.begin(); it != scene->meshlist.end(); it++)
	{
		Mesh *m = (*it);
		aabb_t *m_aabbs = m->GetAABBs();

		for(int i=0; i < m->NumTriangles(); i++)
		{
			waabb.Union(&m_aabbs[i]);
		}		
	}
}

void KDBuildJob::ResetMinMaxBin(minmaxbin_t *mmb, int nbins, int index)
{
	if(nbins != numbins[index])
	{
		if(numbins[index] > 0)
		{
			#ifdef _CELL
				_free_align(minbins[index]);
				_free_align(maxbins[index]);
			#else
				delete [] minbins[index];
				delete [] maxbins[index];
			#endif
		}

		numbins[index] = nbins;

		#ifdef _CELL
			minbins[index] = (bin_t*)_malloc_align(numbins[index] * sizeof(bin_t), 7);
			maxbins[index] = (bin_t*)_malloc_align(numbins[index] * sizeof(bin_t), 7);			
		#else
			minbins[index] = new bin_t[numbins[index]];
			maxbins[index] = new bin_t[numbins[index]];
		#endif
	}

	int i;

	#ifdef _CELL
	vector float zero = spu_splats(0.0f);
	vector float *vminbins = (vector float*)minbins[index];
	vector float *vmaxbins = (vector float*)maxbins[index];

	for(i=0; i < numbins[index]; i++)
	{
		vminbins[i] = zero;
		vmaxbins[i] = zero;		
	}

	#else
	for(i=0; i < numbins[index]; i++)
	{
		minbins[index][i].b[0] = maxbins[index][i].b[0] = 0;
		minbins[index][i].b[1] = maxbins[index][i].b[1] = 0;
		minbins[index][i].b[2] = maxbins[index][i].b[2] = 0;
	}
	#endif

	mmb->minbins = minbins[index];
	mmb->maxbins = maxbins[index];


	mmb->numbins = numbins[index];
	mmb->bestcost = 1000000;
}

inline int GetBin(float left, float right, float pos, int nbins)
{
	if(pos >= right) pos = right; 
	if(pos <= left)  pos = left;

	float width = fabs(right-left);
	float delta = width/(nbins);

	float bin = fabs(pos-left) / delta;
	
	int ibin = (int)bin;

	if(ibin > (nbins-1)) ibin = nbins-1;
	if(ibin < 0) ibin = 0;

	return ibin;
}

void MinMaxBinCount(aabb_t *aabb, minmaxbin_t *mmb, aabb_t *baabb, int a)
{
	int minindex = GetBin(baabb->min[a], baabb->max[a], aabb->min[a], nsamplepoints);
	int maxindex = GetBin(baabb->min[a], baabb->max[a], aabb->max[a], nsamplepoints);

	mmb->minbins[minindex].b[a]++;
	mmb->maxbins[maxindex].b[a]++;
}

#ifdef _CELL
const unsigned int ABS_MASK	= 0x7FFFFFFF;
vector float abs_mask = { *(float*)&ABS_MASK, *(float*)&ABS_MASK, *(float*)&ABS_MASK, *(float*)&ABS_MASK };

inline vector float spu_abs(vector float v)
{
	return spu_and(v, abs_mask);
}


inline vector int GetBinSIMD(vector float left, vector float right, vector float pos, vector float invdelta, vector float nbins)
{
	pos = spu_min(pos, right);
	pos = spu_max(pos, left);
	
	vector float bin = spu_mul(spu_abs(spu_sub(pos, left)), invdelta);

	bin = spu_min(bin, nbins);
	bin = spu_max(spu_splats(0.0f), bin);

	return spu_convts(bin, 0);
}

inline void MinMaxBinCount3SIMD(aabb_t *aabb, minmaxbin_t *mmb, aabb_t *baabb)
{
	vector float *baabb_min = (vector float*)baabb->min;
	vector float *baabb_max = (vector float*)baabb->max;
	vector float *aabb_min = (vector float*)aabb->min;
	vector float *aabb_max = (vector float*)aabb->max;

	vector float nbins = spu_splats((float)nsamplepoints);
	vector float invnbins = spu_re(nbins);
	nbins = spu_sub(nbins, spu_splats(1.0f));

	vector float width = spu_abs(spu_sub(*baabb_max, *baabb_min));
	vector float invdelta = spu_re(spu_mul(width, invnbins));

	vector int minindex = GetBinSIMD(*baabb_min, *baabb_max, *aabb_min, invdelta, nbins);
	vector int maxindex = GetBinSIMD(*baabb_min, *baabb_max, *aabb_max, invdelta, nbins);

	mmb->minbins[minindex[0]].b[0]++;
	mmb->minbins[minindex[1]].b[1]++;
	mmb->minbins[minindex[2]].b[2]++;

	mmb->maxbins[maxindex[0]].b[0]++;
	mmb->maxbins[maxindex[1]].b[1]++;
	mmb->maxbins[maxindex[2]].b[2]++;
}



#endif


void MinMaxBinCount(aabb_t *aabb, minmaxbin_t *mmb, aabb_t *baabb, int axis, int numaxises)
{
	#ifdef _CELL
	if(numaxises == 3 && GetScene()->kdbuild_kernel == K_KDBUILD_SIMD)
	{
		MinMaxBinCount3SIMD(aabb, mmb, baabb);
	}
	else
	#endif
	for(int a = 0; a < numaxises; a++)
		MinMaxBinCount(aabb, mmb, baabb,  (axis+a) % 3);	
}


static float SAHCost(float area, float ctravers, float cleft, float aleft, float cright, float aright)
{
	return ctravers + (cleft * (aleft/area)) + (cright * (aright/area));
}


void MinMaxBinFindBest(minmaxbin_t *mmb, kdbuffer_t *result, int a)
{
	int i;

	for(i=1; i < mmb->numbins; i++)
	{
		int j = mmb->numbins - i - 1;

		mmb->minbins[i].b[a] += mmb->minbins[i-1].b[a];
		mmb->maxbins[j].b[a] += mmb->maxbins[j+1].b[a]; 
	}

	AABB aabb(&result->baabb);

	float width = fabs(result->baabb.max[a] - result->baabb.min[a]);
	float delta = width/(mmb->numbins);
	float x = result->baabb.min[a] + delta;

	for(i=0; i < mmb->numbins-1; i++)
	{
		float aleft, aright;

		aabb.AreaLeftRight(x, a, &aleft, &aright);

		float cost = SAHCost(aabb.Area(), 2, mmb->minbins[i].b[a], aleft, mmb->maxbins[i+1].b[a], aright);

		if(cost < mmb->bestcost)
		{
			result->plane = x;
			result->axis = a;
			result->left_size = (int)mmb->minbins[i].b[a];
			result->right_size = (int)mmb->maxbins[i+1].b[a];

			mmb->bestcost = cost;
		}

		x += delta;	
	}


}

#ifdef _CELL

inline void AreaLeftRight(vector float aabb_min, vector float aabb_max, vector float side, vector float plane, vector float *aleft, vector float *aright)
{
	vector float lside;
	vector float rside;

	lside = spu_abs(spu_sub(plane, aabb_min));
	rside = spu_abs(spu_sub(aabb_max, plane));

	*aleft = spu_mul(lside, side);
	*aright = spu_mul(rside, side);
}


inline vector float SAHCostSIMD(vector float invarea, vector float ctravers, vector float cleft, vector float aleft, vector float cright, vector float aright)
{
	vector float l = spu_mul(cleft, spu_mul(aleft, invarea));
	vector float r = spu_mul(cright, spu_mul(aright, invarea));

	return spu_add(ctravers, spu_add(l, r));
}



void MinMaxBinFindBest3SIMD(minmaxbin_t *mmb, kdbuffer_t *result)
{
	int i;

	for(i=1; i < mmb->numbins; i++)
	{
		int j = mmb->numbins - i - 1;

		vector float *min = (vector float *)mmb->minbins[i].b;
		vector float *max = (vector float *)mmb->maxbins[j].b;

		min[0] = spu_add(min[0], min[-1]);
		max[0] = spu_add(max[0], max[1]);
	}

	vector float *vmax = (vector float*)result->baabb.max;
	vector float *vmin = (vector float*)result->baabb.min;

	vector float vwidth = spu_abs( spu_sub(*vmax, *vmin) );

	vector float vnumbins = spu_splats(1/(float)mmb->numbins);
	vector float vdelta = spu_mul(vwidth, vnumbins);
	vector float vx = spu_add(*vmin, vdelta);

	vector float vside = { vwidth[1] * vwidth[2], vwidth[0] * vwidth[2], vwidth[0] * vwidth[1], 0 };
	vector float invarea = spu_splats( 1/(vwidth[0] * vside[0]));
	vector float vctravers = spu_splats(2.0f);
	vector float vbestcost = spu_splats(mmb->bestcost);
	vector int vbesti = spu_splats(0);
	vector float vbestx = vx;

	for(i=0; i < mmb->numbins-1; i++)
	{
		vector float aleft, aright;

		AreaLeftRight(*vmin, *vmax, vside, vx, &aleft, &aright);

		vector float *vminbin = (vector float *)mmb->minbins[i].b;
		vector float *vmaxbin = (vector float *)mmb->maxbins[i+1].b;

		vector float cost = SAHCostSIMD(invarea, vctravers, *vminbin, aleft, *vmaxbin, aright);

		vector unsigned int cmp = spu_cmpgt(cost, vbestcost);
		vbestcost = spu_sel(cost, vbestcost, cmp);
		vbesti = spu_sel(spu_splats(i), vbesti, cmp);
		vbestx = spu_sel(vx, vbestx, cmp);

		vx = spu_add(vx, vdelta);	
	}	

	int axis = 0;
	float bestcost = vbestcost[axis];

	if(vbestcost[1] < bestcost)
	{
		axis = 1;
		bestcost = vbestcost[1];
	}

	if(vbestcost[2] < bestcost)
	{
		axis = 2;
		bestcost = vbestcost[2];
	}

	int index = vbesti[axis];

	result->plane = vbestx[axis];
	result->axis = axis;
	result->left_size = (int)mmb->minbins[ index ].b[axis];
	result->right_size = (int)mmb->maxbins[ index+1 ].b[axis];
	
	mmb->bestcost = vbestcost[axis];
}
#endif

void MinMaxBinFindBest(minmaxbin_t *mmb, kdbuffer_t *result, int axis, int numaxises)
{
	#ifdef _CELL
		if(numaxises == 3 && GetScene()->kdbuild_kernel == K_KDBUILD_SIMD)
		{
			MinMaxBinFindBest3SIMD(mmb, result);
		}
		else
	#endif
		for(int a = 0; a < numaxises; a++)
			MinMaxBinFindBest(mmb, result, (axis+a) % 3);	
}


void KDBuildJob::KDPartition(kdbuffer_t *in, kdbuffer_t *left, kdbuffer_t *right)
{
	minmaxbin_t lmmb;
	minmaxbin_t rmmb;

	
	// Compute left and right AABB
	memcpy(&left->baabb, &in->baabb, sizeof(aabb_t));
	memcpy(&right->baabb, &in->baabb, sizeof(aabb_t));

	left->baabb.max[ in->axis ] = in->plane;
	right->baabb.min[ in->axis ] = in->plane;

	ResetMinMaxBin(&lmmb, nsamplepoints, 0);
	ResetMinMaxBin(&rmmb, nsamplepoints, 1);

	left->count = 0;
	right->count = 0;	

	splitaxis = (in->axis+1) % 3;

	// Partition and count MinMaxBin
	for(int i=0; i < in->size; i++)
	{
		float min = in->aabb[i].min[in->axis];
		float max = in->aabb[i].max[in->axis];
	
		if(min < in->plane)
		{
			// left
			memcpy(&left->aabb[left->count++], &in->aabb[i], sizeof(aabb_t));
			MinMaxBinCount(&in->aabb[i], &lmmb, &left->baabb, splitaxis, nsplitaxises);
		}

		if(max > in->plane)
		{
			// right
			memcpy(&right->aabb[right->count++], &in->aabb[i], sizeof(aabb_t));
			MinMaxBinCount(&in->aabb[i], &rmmb, &right->baabb, splitaxis, nsplitaxises);
		}
	}

	if(left->count > in->left_size+50)
	{
		cout << "Fatal Error left count > size+50: " << left->count << " > " << in->left_size+50 << endl;
		exit(0);
	}

	if(right->count > in->right_size+50)
	{
		cout << "Fatal Error right count > size+50: " << right->count << " > " << in->right_size+50 << endl;
		exit(0);
	}
	
	left->size = left->count;
	right->size = right->count;
	left->depth = in->depth+1;
	right->depth = in->depth+1;

	
	MinMaxBinFindBest(&lmmb, left, splitaxis, nsplitaxises);
	MinMaxBinFindBest(&rmmb, right, splitaxis, nsplitaxises);	
}

// MakeLeaf
void KDBuildJob::MakeLeaf(kdbuffer_t *kdb)
{
	int n = kdb->node;
	
	leaves[curleaf] = n;

	nodes[n].left = KD_LEAF;
	nodes[n].leafid = curleaf;	

	curleaf++;

	nodes[n].numpolys = kdb->size;
	numleafpolys += kdb->size;

	if(kdb->size > 0)
	{
		nodes[n].polys = (kdpoly_t*)BufferAllocate(polybuffer, nodes[n].numpolys);

	
		for(int i=0; i < kdb->size; i++)
		{
			aabb_t *a = &kdb->aabb[i];

			memcpy(&nodes[n].polys[i].triangle, a->poly->triangle, sizeof(triangle_t));
			memcpy(&nodes[n].polys[i].normal, a->poly->normal, sizeof(normal_t));
			memcpy(&nodes[n].polys[i].color, a->poly->color, sizeof(float[4]));
		}
	}
	
}



void LoadPolygons(Scene *scene, aabb_t *worldaabb)
{
	if(numpolys > 0)
	{
		#ifdef _CELL
			_free_align(polys);
		#else
			delete [] polys;
		#endif

		numpolys = 0;	
	}
	
	vector<Mesh*>::iterator it;

	numpolys = 0;

	for(it = scene->meshlist.begin(); it != scene->meshlist.end(); it++)
	{
		Mesh *m = (*it);

		numpolys += m->NumTriangles();
	}

	#ifdef _CELL
		polys = (kdpolyp_t*)_malloc_align(sizeof(kdpolyp_t) * numpolys, 7);
	#else
		polys = new kdpolyp_t[numpolys];
	#endif

	int index = 0;

	AABB waabb(worldaabb);

	waabb.Reset();

	for(it = scene->meshlist.begin(); it != scene->meshlist.end(); it++)
	{
		Mesh *m = (*it);
		triangle_t *triangles = m->GetTriangles();
		normal_t *normals = m->GetNormals();
		color_t *colors = m->GetColors();
		aabb_t *m_aabbs = m->GetAABBs();

		for(int i=0; i < m->NumTriangles(); i++)
		{
			polys[index].triangle = &triangles[i];
			polys[index].normal = &normals[i];
			polys[index].color = colors[i].c;
		
			m_aabbs[i].poly = &polys[index];

			waabb.Union(&m_aabbs[i]);

			index++;
		}		
	}
}

kdleafpoly_t *GetKDLeafPolysAll()
{
	return leafpolys;
}

void KDMakeLeafPolys()
{
	for(int i=0; i < maxnumleaves; i++)
	{
		KDGetLeafPolys(i, &leafpolys[i].polys, &leafpolys[i].numpolys);	
	
		// cout << "leaf " << i << " n " << leaves[i] << " polys " << (void*)leafpolys[i].polys << " numpolys " << leafpolys[i].numpolys << endl;
	}

}

void Scene::KDInitBuild(int maxleafsize, int extradepth, bool loadpolys)
{
	if(loadpolys)
		LoadPolygons(this, &worldaabb);
	else
		ComputeWorldAABB(&worldaabb, this);

	if(numnodes > 0)
	{
		#ifdef _CELL
			_free_align(nodes);
		#else
			delete [] nodes;
		#endif
	
		#ifdef _CELL
			_free_align(leafpolys);
			_free_align(leaves);		
		#else
			delete [] leafpolys;
			delete [] leaves;
		#endif

		numnodes = 0;
	}	

	if(numpolys < maxleafsize)
		maxnumleaves = numpolys;
	else
		maxnumleaves = numpolys / maxleafsize;
	
	maxdepth = (int)(log2(maxnumleaves) + 0.5 + extradepth);
	maxnumleaves = (int)pow(2.0f, maxdepth);
	numnodes = (int)pow(2.0f, maxdepth+1) - 1;

	#ifdef _CELL
		leafpolys = (kdleafpoly_t*)_malloc_align(sizeof(kdleafpoly_t) * maxnumleaves, 4);
		leaves = (int*)_malloc_align(maxnumleaves*sizeof(int), 7);
	#else
		leafpolys = new kdleafpoly_t[maxnumleaves];
		leaves = new int[maxnumleaves];
	#endif

	for(int i=0; i < maxnumleaves; i++)
		leaves[i] = 0;
	
	numleaves = maxnumleaves;
	numleafpolys = 0;
	
	#ifdef _CELL
		nodes = (kdnode_t*)_malloc_align(numnodes*sizeof(kdnode_t), 7);
	#else
		nodes = new kdnode_t[numnodes];
	#endif
}

void Scene::KDLoadMeshes(KDBuildJob *job, buffer_t *partitionbuffer, buffer_t *leafbuffer, int maxleafsize, int *curnode)
{
	minmaxbin_t mmb;
	kdbuffer_t kdb  ALIGNED(16);

	// Allocate and copy AABBs and compute MinMaxBin
	KDBufferAllocate(&kdb, numpolys);

	job->ResetMinMaxBin(&mmb, nsamplepoints, 0);
	mempcpy(&kdb.baabb, &worldaabb, sizeof(aabb_t));

	vector<Mesh*>::iterator it;

	int index = 0;

	for(it = meshlist.begin(); it != meshlist.end(); it++)
	{
		Mesh *m = (*it);
		aabb_t *maabb = m->GetAABBs();

		for(int i=0; i < m->NumTriangles(); i++)
		{
			MinMaxBinCount(&maabb[i], &mmb, &kdb.baabb,  job->splitaxis, nsplitaxises);			
			memcpy(&kdb.aabb[index], &maabb[i], sizeof(aabb_t));
			kdb.aabb[index].poly = &polys[index];
			index++;		
		}
	}

	kdb.count = numpolys;
	kdb.depth = 0;
	kdb.node = (*curnode)++;
	
	if(numpolys <= maxleafsize)
	{
		BufferCopyTo(leafbuffer, &kdb, 1);
	}
	else
	{
		MinMaxBinFindBest(&mmb, &kdb, job->splitaxis, nsplitaxises);	
		BufferCopyTo(partitionbuffer, &kdb, 1);
	}
}

#ifdef _CELL

void KDBuildJob::KDBuildSPU(int maxleafsize, int extradepth, bool root)
{
	int total_leaf_size ALIGNED(16);

	int bsize = (numpolys * 10) + (maxnumleaves * 50);
	int a_curnode ALIGNED(16);
	int a_numleafpolys ALIGNED(16);

	a_numleafpolys = 0;
	int b = 0;

	kdbuild_arg_t arg ALIGNED(16);

	if(GetScene()->kdbuild_kernel == K_KDBUILD_SIMD_SPU)
		arg.simd = 1;
	else
		arg.simd = 0;


	arg.nsamplepoints = nsamplepoints;
	arg.nsplitaxises = nsplitaxises;
	arg.curnode = curnode;
	arg.curleaf = curleaf;
	arg.pcurnode = &a_curnode;
	arg.maxdepth = maxdepth;
	arg.maxleafsize = maxleafsize;
	arg.nodes = nodes;
	arg.ptotal_leaf_size = &total_leaf_size;
	arg.tid = tid;

	//buffer_t tmp_leaf_aabb_buffer ALIGNED(16);
	// buffer_t tmp_leafbuffer ALIGNED(16);

	arg.pleaf_aabb_buffer = leaf_aabb_buffer;
	arg.pleafbuffer = leafbuffer;
	arg.pcurleaf = &curleaf;
	arg.pleaves = leaves;
	arg.numleafpolys = &a_numleafpolys;

	memcpy(&arg.aabb_buffer[0], aabb_buffer[0], sizeof(buffer_t));
	memcpy((void*)&arg.aabb_buffer[1], aabb_buffer[1], sizeof(buffer_t));

	memcpy(&arg.kdbuffer[0], kdbuffer[0], sizeof(buffer_t));
	memcpy(&arg.kdbuffer[1], kdbuffer[1], sizeof(buffer_t));

	memcpy(&arg.leaf_aabb_buffer, leaf_aabb_buffer, sizeof(buffer_t));
	memcpy(&arg.leafbuffer, leafbuffer, sizeof(buffer_t));

	memcpy(&arg.polybuffer, polybuffer, sizeof(buffer_t));

	arg.njobs = njobs;

	for(int i=0; i < njobs; i++)
	{
		memcpy(&arg.job_kdbuffer[i], jobs[i]->kdbuffer[0], sizeof(buffer_t));
		memcpy(&arg.job_aabb_buffer[i], jobs[i]->aabb_buffer[0], sizeof(buffer_t));
		memcpy(&arg.job_leaf_aabb_buffer[i], jobs[i]->leaf_aabb_buffer, sizeof(buffer_t));
		memcpy(&arg.job_leafbuffer[i], jobs[i]->leafbuffer, sizeof(buffer_t));

		arg.pjob_leafbuffer[i] = jobs[i]->leafbuffer;
		arg.pjob_kdbuffer[i] = jobs[i]->kdbuffer[0];

		arg.sema[i] = (void*)&jobs[i]->sem;
	}

	// Load and Run SPE program
	SPEProgram spe_kdbuild(&kdbuild);
	spe_kdbuild.Run(GetSPEContext(tid), &arg);	

	// Alignment fix
	curnode = a_curnode;
	numleafpolys = a_numleafpolys;

	int size = BufferNumElements(leafbuffer);

	numleaves = size;
	curleaf = size;
}
#endif

void KDBuildJob::KDBuild(int maxleafsize, int extradepth, bool root)
{
	kdbuffer_t l_kdb;
	kdbuffer_t r_kdb;

	int b = 0;

	if(BufferEmpty(kdbuffer[b]))
	{
		for(int i=0; i < njobs; i++)
			sem_post(&jobs[i]->sem);
	}

	// Make nodes
	while(! BufferEmpty(kdbuffer[b]) )
	{

		kdbuffer_t *kdb = (kdbuffer_t*)kdbuffer[b]->buffer;
		int size = BufferNumElements(kdbuffer[b]);
		int i;
		
		BufferClear(aabb_buffer[1-b]);
		BufferClear(kdbuffer[1-b]);

		for(i=0; i < size; i++)
		{
			l_kdb.node = curnode++;
			r_kdb.node = curnode++;		

			nodes[ kdb[i].node ].split = kdb[i].plane;
			nodes[ kdb[i].node ].axis =  kdb[i].axis;
			nodes[ kdb[i].node ].left =  l_kdb.node;
			nodes[ kdb[i].node ].right = r_kdb.node;				

			KDBufferAllocate(&l_kdb, kdb[i].left_size, aabb_buffer[1-b]);

			if(curjob < njobs)
				KDBufferAllocate(&r_kdb, kdb[i].right_size, jobs[curjob]->aabb_buffer[0]);
			else
				KDBufferAllocate(&r_kdb, kdb[i].right_size, aabb_buffer[1-b]);

			KDPartition(&kdb[i], &l_kdb, &r_kdb);

			if(l_kdb.depth == maxdepth || l_kdb.size <= maxleafsize)
			{
				l_kdb.aabb = (aabb_t*)BufferCopyTo(leaf_aabb_buffer, l_kdb.aabb, l_kdb.count);
				BufferCopyTo(leafbuffer, &l_kdb, 1);
			}			
			else
				BufferCopyTo(kdbuffer[1-b], &l_kdb, 1);

			if(r_kdb.depth == maxdepth || r_kdb.size <= maxleafsize)
			{
				if(curjob < njobs)
				{
					r_kdb.aabb = (aabb_t*)BufferCopyTo(jobs[curjob]->leaf_aabb_buffer, r_kdb.aabb, r_kdb.count);
					BufferCopyTo(jobs[curjob]->leafbuffer, &r_kdb, 1);
				}
				else
				{
					r_kdb.aabb = (aabb_t*)BufferCopyTo(leaf_aabb_buffer, r_kdb.aabb, r_kdb.count);
					BufferCopyTo(leafbuffer, &r_kdb, 1);
				}
			}
			else
			{
				if(curjob < njobs)
					BufferCopyTo(jobs[curjob]->kdbuffer[0], &r_kdb, 1);
				else
					BufferCopyTo(kdbuffer[1-b], &r_kdb, 1);
			}
	
		}
	
		if(curjob < njobs)
		{
			// Start other job
			sem_post(&jobs[curjob]->sem);
			curjob++;
		}

		b =  1 - b;

	}

	// Check if still remaining jobs and start them
	while(curjob < njobs)
	{
		// Start other job
		sem_post(&jobs[curjob]->sem);
		curjob++;
	}

	
	// Make leaves
	kdbuffer_t *kdb = (kdbuffer_t*)leafbuffer->buffer;
	int size = BufferNumElements(leafbuffer);

	//cout << "Make leaves " << size << endl;

	int i;

	for(i=0; i < size; i++)
	{
		MakeLeaf(&kdb[i]);
	}

	numleaves = size;
	curleaf = size;
}


