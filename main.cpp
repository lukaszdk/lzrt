#include <iostream>
#include <sstream>
#include <fstream>
#include <pthread.h>
#include <SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h> 
#include <imagebuffer/framebuffer.h>
#include <scene/camera.h>
#include <scene/scene.h>
#include <share/structs.h>
#include <util/timer.h>
#include <util/stats.h>
#include <util/luabind.h>
#ifdef _CELL
	#include <spu/speprogram.h>
#endif


#ifdef __i386 
	#define ARCH_STR	"i386"
#else
	#ifdef __powerpc
		#define ARCH_STR	"PowerPC"
	#else
		#define ARCH_STR	"Unknown arch"
	#endif
#endif


using namespace std;
using namespace luabind;

#define BPP				8	
#define MOVE_SPEED 		1
#define RENDER_LINES	8

#define SCREEN_WIDTH	512
#define SCREEN_HEIGHT	512


extern int KDGetNumLeafPolys();
extern int KDGetNumLeaves();
extern int KDGetNumberOfSplits();
extern void KDIncNumberOfSplits();
extern void KDDecNumberOfSplits();
extern int KDGetNumberOfAxises();
extern void KDChangeNumberOfAxises();
extern int KDGetMaxDepth();
extern void InitPPECallbacks();
extern int GetFrustrumDEP();
extern int GetNumFrustrum();
extern int GetNumFrustrumHits();
extern int GetFrustumTotalDEP();

static struct
{
	char luascript[512];
	char luapath[512];
	int width;
	int height;
	bool verbose;
	int renderer;
	bool fullscreen;
	bool animate;
	char *lwo;
	int nthreads;
	int njobs;
	bool printstats;
	bool printsummary;
} args;

static struct
{
	float x;
	float y;
	float z;
} camerapos;

static struct
{
	float min_fps;
	float max_fps;
	float min_tup;
	float max_tup;
	int min_als;
	int max_als;
	int min_lhp;
	int max_lhp;
	float min_lpr;
	float max_lpr;
	int min_apt;
	int max_apt;
	int min_fhp;
	int max_fhp;
	int min_dep;
	int max_dep;
	float min_aep;
	float max_aep;
	float min_mpt;
	float max_mpt;
	unsigned int min_tt;
	unsigned int max_tt;
	unsigned int min_kdb;
	unsigned int max_kdb;
	unsigned int min_rg;
	unsigned int max_rg;
	unsigned int min_kdt;
	unsigned int max_kdt;
	unsigned int min_pp;
	unsigned int max_pp;
	unsigned int min_pt;
	unsigned int max_pt;	
} stats;

Uint32 rendertime;
static Timer timer;
bool run = true;
bool firstframe = true;
bool verbose_stats = false;
ThreadPool *threadpool;
int frame = 0;

void SetNumRayTraceJobs(int n)
{
	args.njobs = n;
}


void SDLevent()
{
	SDL_Event event;

	while ( SDL_PollEvent(&event) ) 
	{
		switch (event.type) 
		{
			case SDL_KEYDOWN:
			{
				if( event.key.keysym.sym == SDLK_UP) camerapos.z = -MOVE_SPEED;
				if( event.key.keysym.sym == SDLK_DOWN) camerapos.z = MOVE_SPEED; 
				if( event.key.keysym.sym == SDLK_LEFT) camerapos.x = -MOVE_SPEED;
				if( event.key.keysym.sym == SDLK_RIGHT) camerapos.x = MOVE_SPEED; 				

				if( event.key.keysym.sym == SDLK_1)	GetScene()->MeshTransformNext();
				if( event.key.keysym.sym == SDLK_2) GetScene()->KDBuildNext();
				if( event.key.keysym.sym == SDLK_3)	GetScene()->RayGenNext();
				if( event.key.keysym.sym == SDLK_4)	GetScene()->KDTraversNext();
				if( event.key.keysym.sym == SDLK_5)	GetScene()->PolyPartitionNext();
				if( event.key.keysym.sym == SDLK_6) GetScene()->PolyTestNext();

				if( event.key.keysym.sym == SDLK_b) 
				{
					GetScene()->RebuildKDTree(threadpool, true);
				}

				if( event.key.keysym.sym == SDLK_r) 
				{
					KDIncNumberOfSplits();
					GetScene()->RebuildKDTree(threadpool);
				}

				if( event.key.keysym.sym == SDLK_t) 
				{
					KDDecNumberOfSplits();
					GetScene()->RebuildKDTree(threadpool);
				}	

				if( event.key.keysym.sym == SDLK_g)
				{
					KDChangeNumberOfAxises();
					GetScene()->RebuildKDTree(threadpool);
				} 

				if( event.key.keysym.sym == SDLK_a) 
				{
					if(GetScene()->raster)
						GetScene()->raster = false;
					else
						GetScene()->raster = true;
				}


				if( event.key.keysym.sym == SDLK_s) 
				{
					if(GetScene()->shadowrays)
						GetScene()->shadowrays = false;
					else
						GetScene()->shadowrays = true;
				}

				if( event.key.keysym.sym == SDLK_d) 
				{
					if(GetScene()->shading)
						GetScene()->shading = false;
					else
						GetScene()->shading = true;
				}

				if( event.key.keysym.sym == SDLK_v) 
				{
					if(verbose_stats)
						verbose_stats = false;
					else
						verbose_stats = true;

				}

				if( event.key.keysym.sym == SDLK_x) 
				{
					if(GetScene()->animate)
						GetScene()->animate = false;
					else
						GetScene()->animate = true;
				}

				if( event.key.keysym.sym == SDLK_e) 
				{
					if(GetScene()->secondrays)
						GetScene()->secondrays = false;
					else
						GetScene()->secondrays = true;
				}


				if( event.key.keysym.sym == SDLK_n) 
				{
					if(args.njobs > 1)
					{
						args.njobs /= 2;
					}
				}
				
				if( event.key.keysym.sym == SDLK_m) 
				{
					args.njobs *= 2;
				}

				if( event.key.keysym.sym == SDLK_h) 
				{
					if(args.nthreads > 1)
					{
						args.nthreads--;
					}
				}
				
				if( event.key.keysym.sym == SDLK_j) 
				{
					#ifdef _CELL
					if(args.nthreads < NumSPEs())
					#else	
					if(args.nthreads < 8)
					#endif
						args.nthreads++;
				}

				if( event.key.keysym.sym == SDLK_y) 
				{
					GetScene()->FrustumDecDim();
				}

				if( event.key.keysym.sym == SDLK_u) 
				{
					GetScene()->FrustumIncDim();
				}
				
		
				if( event.key.keysym.sym == SDLK_i) 
				{
					GetScene()->FrustumDecNumSteps();
				}

				if( event.key.keysym.sym == SDLK_o) 
				{
					GetScene()->FrustumIncNumSteps();
				}
				

				if( event.key.keysym.sym == SDLK_k) 
				{
					GetScene()->FrustumDecStepsize();
				}

				if( event.key.keysym.sym == SDLK_l) 
				{
					GetScene()->FrustumIncStepsize();
				}
								



				if( event.key.keysym.sym == SDLK_q) 
				{
					args.renderer = (args.renderer+1) % R_NUM;
				
					#ifdef _CELL
					if(args.renderer == R_CELL)
					{
						GetScene()->meshtransform_kernel = K_MESHTRANSFORM_SIMD_SPU;
						GetScene()->kdbuild_kernel = K_KDBUILD_SIMD_SPU;
						GetScene()->raygen_kernel = K_RAYGEN_F_SIMD_SPU;
						GetScene()->kdtravers_kernel = K_KDTRAVERS_SPU;	
						GetScene()->polypartition_kernel = K_POLYPARTITION_SPU;
						GetScene()->polytest_kernel = K_POLYTEST_SIMD_SPU;
					}
					else
					{
					#endif
						GetScene()->meshtransform_kernel = K_MESHTRANSFORM;	
						GetScene()->kdbuild_kernel = K_KDBUILD;
						GetScene()->raygen_kernel = K_RAYGEN_FRUSTUM;
						GetScene()->kdtravers_kernel = K_KDTRAVERS;
						GetScene()->polypartition_kernel = K_POLYPARTITION;
						GetScene()->polytest_kernel = K_POLYTEST;
					#ifdef _CELL
					}
					#endif
	
				}		

				if( event.key.keysym.sym == SDLK_ESCAPE) run = false;


			}
			break;
			
			case SDL_KEYUP:
			{
				if( event.key.keysym.sym == SDLK_UP)  camerapos.z = 0.0f;
				if( event.key.keysym.sym == SDLK_DOWN) camerapos.z = 0.0f; 
				if( event.key.keysym.sym == SDLK_LEFT) camerapos.x = 0.0f;
				if( event.key.keysym.sym == SDLK_RIGHT) camerapos.x = 0.0f; 

			} break;




			case SDL_QUIT:
				run = false;
			break;

			default:
			break;
		}
	}
}

bool fileExists(const char *filename)
{
	struct stat stFileInfo;
	int intStat;

	// Attempt to get the file attributes
	intStat = stat(filename, &stFileInfo); 

	if(intStat != 0)
		return false;
	else
		return true;
}

char* filePath()
{
	return args.luapath;
}

int parseArgs(int argc, char *argv[])
{
	if(argc == 1)
	{
		cout << "lzrt [options] [Lua scene file]" << endl;
		cout << "Options:" << endl;
		#ifdef _CELL
		cout << " -c         : Cell renderer (default)" << endl;
		cout << " -g         : Generic renderer" << endl;
		#else
		cout << " -g         : Generic renderer (default)" << endl;
		#endif
		cout << " -t <num>   : Initial number of threads" << endl;
		cout << " -j <num>   : Initial number of jobs, must be 2^n" << endl;
		cout << " -w <num>   : Window width, default " << args.width << endl;
		cout << " -h <num>   : Window height, default " << args.height << endl;
		cout << " -p         : Print LaTeX stats to stats.tex" << endl;
		cout << " -s         : Print summary stats" << endl;
		cout << " -f         : Full screen" << endl;
		cout << " -a         : Do not start animation" << endl;
		
		return 1;
	}
	else
	{
		int i=0;
		
		for(int i=1; i < argc-1; i++)
		{
			if(strncmp(argv[i], "-w", 2) == 0 && argc > i)
				args.width = atoi(argv[i+1]);

			if(strncmp(argv[i], "-h", 2) == 0 && argc > i)
				args.height = atoi(argv[i+1]);

			if(strncmp(argv[i], "-t", 2) == 0 && argc > i)
				args.nthreads = atoi(argv[i+1]);

			if(strncmp(argv[i], "-j", 2) == 0 && argc > i)
				args.njobs = atoi(argv[i+1]);


			if(strncmp(argv[i], "-l", 2) == 0 && argc > i)
				args.lwo = argv[i+1];
	

			#ifdef _CELL
			if(strncmp(argv[i], "-c", 2) == 0 && argc > i)
			{
				args.renderer = R_CELL;
			}
			#endif

			if(strncmp(argv[i], "-g", 2) == 0 && argc > i)
			{
				args.renderer = R_GENERIC;
			}

			
			if(strncmp(argv[i], "-p", 2) == 0 && argc > i)
			{
				args.printstats = true;
			}

			if(strncmp(argv[i], "-s", 2) == 0 && argc > i)
			{
				args.printsummary = true;
			}


			if(strncmp(argv[i], "-f", 2) == 0 && argc > i)
			{
				args.fullscreen = true;
			}	

			if(strncmp(argv[i], "-v", 2) == 0 && argc > i)
			{
				verbose_stats = true;
			}	


			if(strncmp(argv[i], "-a", 2) == 0 && argc > i)
			{
				args.animate = false;
			}	
		}

		strcpy(args.luascript, argv[argc-1]);

		strncpy(args.luapath, args.luascript, (int)(strrchr(args.luascript, '/') - args.luascript));

		if(!args.luascript)
		{
			cout << "Lua file '" << args.luascript << "' doesn't exists" << endl;
			return 1;
		}		
	}

	return 0;
}

void Init()
{
	camerapos.x = 0;
	camerapos.y = 0;
	camerapos.z = 0;

	//stats->fps = 0;

	args.width = SCREEN_WIDTH;
	args.height = SCREEN_HEIGHT;

	args.printstats = false;
	args.printsummary = false;

	args.lwo = 0;
	args.verbose = false;

	#ifdef _CELL
	args.renderer = R_CELL;
	#else
	args.renderer = R_GENERIC;
	#endif	

	args.animate = true;
	args.fullscreen = false;

	#ifdef _CELL
	args.nthreads = 4;
	args.njobs = 4;
	#else
	args.nthreads = 2;
	args.njobs = 2;
	#endif
}

ofstream file;
stringstream kernel_stats;

void PrintSummaryStats(ImageBuffer *imgbuf, Scene *scene)
{
	if(!args.printsummary) return;

	int numleaves = 0;

	for(int i=0; i < scene->nkdjobs; i++)
	{
		numleaves += scene->kdjob[i].curleaf;
	}

	unsigned opt_rendertime =  scene->transform_time +  scene->kdbuild_time;
	opt_rendertime += scene->raygen_time + scene->kdtravers_time;
	opt_rendertime += scene->polypartition_time + scene->polytest_time + scene->raster_time;	


	if(args.printsummary && (firstframe || GetScene()->animate) && frame < 25 && frame > 0)
	{
		int apt; 
		
		if(scene->nleafhits > 0)
			apt = (scene->npolytests)/scene->nleafhits;
		else
			apt = 0;

		int als = KDGetNumLeafPolys()/numleaves;

		int fhp;

		if(scene->nfrustums > 0)
			fhp = (scene->nfrustumhits*100) / scene->nfrustums;
		else
			fhp = 0;

		float aep;

		if(scene->nfrustumhits > 0)
			aep = (float)scene->total_frustum_dep / (float)scene->nfrustumhits;
		else
			aep = 0;

		if(frame == 1)
		{
			stats.min_fps = stats.max_fps = (1000.0f) / (float)(rendertime);
			stats.min_tup = stats.max_tup = opt_rendertime*100 / rendertime;
			stats.min_als = stats.max_als = als;
			stats.min_lhp = stats.max_lhp = (scene->nleafhits*100)/ ( imgbuf->GetWidth() * imgbuf->GetHeight());
			stats.min_lpr = stats.max_lpr =(float)apt/(float)als;
			stats.min_apt = stats.max_apt = apt;
			stats.min_fhp = stats.max_fhp = fhp;
			stats.min_dep = stats.max_dep = scene->frustum_dep;
			stats.min_aep = stats.max_aep = (float)aep;
			stats.min_mpt = stats.max_mpt = (scene->npolytests)/ 1000000.0f;

			stats.min_tt = stats.max_tt = scene->transform_time;			
			stats.min_kdb = stats.max_kdb = scene->kdbuild_time;
			stats.min_rg = stats.max_rg = scene->raygen_time;
			stats.min_kdt = stats.max_kdt = scene->kdtravers_time;
			stats.min_pp = stats.max_pp = scene->polypartition_time;
			stats.min_pt = stats.max_pt = scene->polytest_time;
		}
		else
		{
			stats.min_fps = min( (1000.0f) / (float)(rendertime), stats.min_fps);
			stats.min_tup = min( (float)(opt_rendertime*100 / rendertime), stats.min_tup);
			stats.min_als = min( als, stats.min_als);
			stats.min_lhp = min( (scene->nleafhits*100)/ ( imgbuf->GetWidth() * imgbuf->GetHeight()), (unsigned int)stats.min_lhp );
			stats.min_lpr = min( (float)apt/(float)als, stats.min_lpr);
			stats.min_apt = min( apt, stats.min_apt);
			stats.min_fhp = min( fhp, stats.min_fhp);
			stats.min_dep = min( scene->frustum_dep, (unsigned int)stats.min_dep);
			stats.min_aep = min( (float)aep, stats.min_aep);

			stats.min_mpt = min( (scene->npolytests)/ 1000000.0f, stats.min_mpt);
			stats.max_mpt = max( (scene->npolytests)/ 1000000.0f, stats.max_mpt);


			stats.max_fps = max( (1000.0f) / (float)(rendertime), stats.max_fps);
			stats.max_tup = max( (float)(opt_rendertime*100 / rendertime), stats.max_tup);
			stats.max_als = max( als, stats.max_als);
			stats.max_lhp = max( (scene->nleafhits*100)/ ( imgbuf->GetWidth() * imgbuf->GetHeight()), (unsigned int)stats.max_lhp );
			stats.max_lpr = max( (float)apt/(float)als, stats.max_lpr);
			stats.max_apt = max( apt, stats.max_apt);
			stats.max_fhp = max( fhp, stats.max_fhp);
			stats.max_dep = max( scene->frustum_dep, (unsigned int)stats.max_dep);
			stats.max_aep = max( (float)aep, stats.max_aep);

			stats.min_tt = min(scene->transform_time, stats.min_tt);
			stats.max_tt = max(scene->transform_time, stats.max_tt);

			stats.min_kdb = min(scene->kdbuild_time, stats.min_kdb);
			stats.max_kdb = max(scene->kdbuild_time, stats.max_kdb);

			stats.min_rg = min(scene->raygen_time, stats.min_rg);
			stats.max_rg = max(scene->raygen_time, stats.max_rg);

			stats.min_kdt = min(scene->kdtravers_time, stats.min_kdt);
			stats.max_kdt = max(scene->kdtravers_time, stats.max_kdt);

			stats.min_pp = min(scene->polypartition_time, stats.min_pp);
			stats.max_pp = max(scene->polypartition_time, stats.max_pp);
	
			stats.min_pt = min(scene->polytest_time, stats.min_pt);
			stats.max_pt = max(scene->polytest_time, stats.max_pt);
		}
	}

	if(frame == 25)
	{
		printf("Summary\n");
		printf("\tThreads: %i\n", args.nthreads); 
		printf("\tJobs   : %i\n\n", args.njobs);  
		printf("\tFPS    : %.2f - %.2f\n", stats.min_fps, stats.max_fps);
		printf("\tMPT    : %.2f - %.2f\n", stats.min_mpt, stats.max_mpt);
		printf("\tTUP    : %g%% - %g%%\n", stats.min_tup, stats.max_tup);
		printf("\tALS    : %i - %i\n", stats.min_als, stats.max_als);
		printf("\tLHP    : %i%% - %i%%\n", stats.min_lhp, stats.max_lhp);
		printf("\tLPR    : %.2g - %.2g\n", stats.min_lpr, stats.max_lpr);
		printf("\tAPT    : %i - %i\n", stats.min_apt, stats.max_apt);
		printf("\tFHP    : %i%% - %i%%\n", stats.min_fhp, stats.max_fhp);
		printf("\tDEP    : %i - %i\n", stats.min_dep, stats.max_dep);
		printf("\tAEP    : %.2g - %.2g\n\n", stats.min_aep, stats.max_aep);

		printf("\tTT     : %i - %i ms\n", stats.min_tt, stats.max_tt);
		printf("\tKDB    : %i - %i ms\n", stats.min_kdb, stats.max_kdb);
		printf("\tRG     : %i - %i ms\n", stats.min_rg, stats.max_rg);
		printf("\tKDT    : %i - %i ms\n", stats.min_kdt, stats.max_kdt);
		printf("\tPP     : %i - %i ms\n", stats.min_pp, stats.max_pp);
		printf("\tPT     : %i - %i ms\n", stats.min_pt, stats.max_pt);
	}

}


void PrintLatexStats(ImageBuffer *imgbuf, Scene *scene)
{

	int numleaves = 0;

	for(int i=0; i < scene->nkdjobs; i++)
	{
		numleaves += scene->kdjob[i].curleaf;
	}

	unsigned opt_rendertime =  scene->transform_time +  scene->kdbuild_time;
	opt_rendertime += scene->raygen_time + scene->kdtravers_time;
	opt_rendertime += scene->polypartition_time + scene->polytest_time + scene->raster_time;

	if(args.printstats && (firstframe || GetScene()->animate) && frame < 25 && frame > 0)
	{
		int apt; 
		
		if(scene->nleafhits > 0)
			apt = (scene->npolytests)/scene->nleafhits;
		else
			apt = 0;

		int als = KDGetNumLeafPolys()/numleaves;

		int fhp;

		if(scene->nfrustums > 0)
			fhp = (scene->nfrustumhits*100) / scene->nfrustums;
		else
			fhp = 0;

		float aep;

		if(scene->nfrustumhits > 0)
			aep = (float)scene->total_frustum_dep / (float)scene->nfrustumhits;
		else
			aep = 0;

		if(frame == 1)
		{
			file.open("stats.tex", ios::trunc);

			file << "\\documentclass{article}" << endl;
			file << "\\usepackage{multirow}" << endl;
			file << "\\usepackage{a4}" << endl;
			file << "\\usepackage[danish]{babel}" << endl;
			file << "\\usepackage[utf8]{inputenc}" << endl;
			file << "\\usepackage{verbatim}" << endl;
			file << "\\begin{document}" << endl;

			file << "\\subsection{" << args.luascript << " " << ARCH_STR << " " << (args.renderer == R_GENERIC ? "Generic" : "Cell") << "}" << endl;
			file << "\\scriptsize" << endl;
			file << "\\subsubsection{Parametre}" << endl;

			file << "\\begin{tabular}{|c|c|c|c|c|c|c|c|c|c|c|c|c|c|c|c|} \\hline" << endl;
			file << "NT & NRJ & SR & RR & MLS & ED & NEP & NEA & FD & FSS & FNS \\\\ \\hline" << endl;
			file << args.nthreads << " & " << args.njobs << " & ";
			file << (scene->shadowrays ? "On" : "Off") << " & ";
			file << (scene->secondrays ? "On" : "Off") << " & ";
			file << scene->kdtree_mls << " & " << scene->kdtree_etd << " & ";
			file << KDGetNumberOfSplits() << " & " << KDGetNumberOfAxises() << " & ";
			file << scene->frustumdim << " & " << scene->frustumstepsize << " & " << scene->frustumstep;

		
			file << "\\\\ \\hline" << endl;
			file << endl;

			file << "\\hline" << endl;
			file << "\\end{tabular}" << endl;

			file << "\\subsubsection{Målinger}" << endl;
			file << "\\begin{tabular}{|l|c|c|c|c|c|c|c|c|c|c|c|c|c|c|c|} \\hline " << endl;
			file << " & FPS & MPT & TUP & ALS & LHP & LPR & APT & FHP & DEP & AEP \\\\ \\hline" << endl;


			kernel_stats << "\\begin{tabular}{|l|c|c|c|c|c|c|c|c|c|c|c|c|c|c|c|} \\hline \n";
			kernel_stats << " & Transform & KDBuild & RayGen & Travers & PolyPartition & PolyTest & Total \\\\ \\hline \n";


		}

		file.precision(2);

		file << "Frame " << frame << " & "; 
		file << (float)(1000.0f) / (float)(rendertime) << " & ";
		file << (scene->npolytests)/ 1000000.0f << " & ";
		file << opt_rendertime*100 / rendertime << "\\% & ";
		file << als << " & ";
		file << (scene->nleafhits*100)/ ( imgbuf->GetWidth() * imgbuf->GetHeight()) << "\\% & ";
		file << (float)apt/(float)als << " & ";
		file << apt << " & ";
		file << fhp << "\\% & ";
		file << scene->frustum_dep << " & ";printf("Total\n");
		printf("\tThreads: %i\n", args.nthreads); 
		printf("\tJobs   : %i\n\n", args.njobs);  
		printf("\tFPS    : %.2f - %.2f\n", stats.min_fps, stats.max_fps);
		printf("\tMPT    : %.2f - %.2f\n", stats.min_mpt, stats.max_mpt);
		printf("\tTUP    : %g%% - %g%%\n", stats.min_tup, stats.max_tup);
		printf("\tALS    : %i - %i\n", stats.min_als, stats.max_als);
		printf("\tLHP    : %i%% - %i%%\n", stats.min_lhp, stats.max_lhp);
		printf("\tLPR    : %.2g - %.2g\n", stats.min_lpr, stats.max_lpr);
		printf("\tAPT    : %i - %i\n", stats.min_apt, stats.max_apt);
		printf("\tFHP    : %i%% - %i%%\n", stats.min_fhp, stats.max_fhp);
		printf("\tDEP    : %i - %i\n", stats.min_dep, stats.max_dep);
		printf("\tAEP    : %.2g - %.2g\n\n", stats.min_aep, stats.max_aep);

		printf("\tTT     : %i - %i ms\n", stats.min_tt, stats.max_tt);
		printf("\tKDB    : %i - %i ms\n", stats.min_kdb, stats.max_kdb);
		printf("\tRG     : %i - %i ms\n", stats.min_rg, stats.max_rg);
		printf("\tKDT    : %i - %i ms\n", stats.min_kdt, stats.max_kdt);
		printf("\tPP     : %i - %i ms\n", stats.min_pp, stats.max_pp);
		printf("\tPT     : %i - %i ms\n", stats.min_pt, stats.max_pt);
		file << aep;
		file << "\\\\ \\hline" << endl;

	

		if(frame > 0)
		{
			kernel_stats.precision(2);

			kernel_stats << "Frame " << frame << " & ";
			kernel_stats << scene->transform_time << " ms & ";
			kernel_stats << scene->kdbuild_time << " ms & ";
			kernel_stats << scene->raygen_time << " ms & ";
			kernel_stats << scene->kdtravers_time << " ms & ";
			kernel_stats << scene->polypartition_time << " ms & ";
			kernel_stats << scene->polytest_time << " ms & ";
			kernel_stats << (scene->transform_time + scene->kdbuild_time + scene->raygen_time + scene->kdtravers_time + scene->polypartition_time + scene->polytest_time) << " ms "; 
			kernel_stats << " \\\\ \\hline\n";


		}
	}

	
	if(args.printstats && frame == 25)
	{

		file << "\\end{tabular}" << endl;

		kernel_stats << "\\end{tabular} \n";


		file << "\\\\ \\ \\\\ \\ \\\\ \\ \\\\\n" << kernel_stats.str();

		file << "\\normalsize" << endl;

		file << "\\end{document}" << endl;
		
		file.close();

		cout << "Done writing stats.tex" << endl;
	}





}

void PrintStats(ImageBuffer *imgbuf, Scene *scene)
{
	PrintLatexStats(imgbuf, scene);
	PrintSummaryStats(imgbuf, scene);

	int linespace = 15;

	if(verbose_stats)
	{	
		// Top left	
		imgbuf->SetPrintXY(10, 10, linespace);
		imgbuf->Printlnf("Rendertime            %5i ms", rendertime);
		
		imgbuf->Printlnf("1: %-19s%5i ms", scene->MeshTransformString(), scene->transform_time);
		imgbuf->Printlnf("2: %-19s%5i ms", scene->KDBuildString(), scene->kdbuild_time);
		imgbuf->Printlnf("3: %-19s%5i ms", scene->RayGenString(), scene->raygen_time);
		imgbuf->Printlnf("4: %-19s%5i ms", scene->KDTraversString(), scene->kdtravers_time);
		imgbuf->Printlnf("5: %-19s%5i ms", scene->PolyPartitionString(), scene->polypartition_time);
		imgbuf->Printlnf("6: %-19s%5i ms", scene->PolyTestString(), scene->polytest_time);
		imgbuf->Printlnf("7: Raster             %5i ms", scene->raster_time);
	}

	int numleaves = 0;

	for(int i=0; i < scene->nkdjobs; i++)
	{
		numleaves += scene->kdjob[i].curleaf;
	}

	unsigned opt_rendertime =  scene->transform_time +  scene->kdbuild_time;
	opt_rendertime += scene->raygen_time + scene->kdtravers_time;
	opt_rendertime += scene->polypartition_time + scene->polytest_time + scene->raster_time;

	// Top right
	imgbuf->SetPrintXY(imgbuf->GetWidth()-90, 10, linespace);
	imgbuf->Printlnf("FPS %5.2f", (1000.0f) / (float)(rendertime));
	
	if(verbose_stats)
	{
		
		imgbuf->Printlnf("TUP %5i%%", opt_rendertime*100 / rendertime);
		imgbuf->Printlnf("");

		// Number of Evaluated axises	
		imgbuf->Printlnf("NEA %5i", KDGetNumberOfAxises());
		// Number of Evaluated positions
		imgbuf->Printlnf("NEP %5i", KDGetNumberOfSplits());
		// World hit pct
		imgbuf->Printlnf("LHP %5i%%", (scene->nleafhits*100)/ ( imgbuf->GetWidth() * imgbuf->GetHeight()));
	
		// Average leafs pr. ray
		int apt; 
		
		if(scene->nleafhits > 0)
			apt = (scene->npolytests)/scene->nleafhits;
		else
			apt = 0;

		int als = KDGetNumLeafPolys()/numleaves;

		imgbuf->Printlnf("LPR %5.2g", (float)apt/(float)als);
		// Average number of polytests
		imgbuf->Printlnf("APT %5i", apt);
		// Aveareg leaf size
		imgbuf->Printlnf("ALS %5i", als);
		// Polygon copy factor
		imgbuf->Printlnf("PCF %5.2g", (float)KDGetNumLeafPolys()/(float)scene->numpolys);
		
		
		imgbuf->Printlnf("KNL %5i", numleaves);


		imgbuf->Printlnf("KDT %5i", KDGetMaxDepth());
		imgbuf->Printlnf("DEP %5i", scene->frustum_dep);

		// average frustum depth EP

		float aep;

		if(scene->nfrustumhits > 0)
			aep = (float)scene->total_frustum_dep / (float)scene->nfrustumhits;
		else
			aep = 0;

		//cout << "total_frustum_dep " << scene->total_frustum_dep << " nfrustumhits " << scene->nfrustumhits << endl;

		imgbuf->Printlnf("AEP %5.2g", (float)aep);

		// frustum hit pct.
		int fhp;

		if(scene->nfrustums > 0)
			fhp = (scene->nfrustumhits*100) / scene->nfrustums;
		else
			fhp = 0;

		// cout << "nfrustumhits " << scene->nfrustumhits << " nfrustums " << scene->nfrustums << endl;

		imgbuf->Printlnf("FHP %5i%%", fhp);

		imgbuf->Printlnf("MPT %5.2g", (scene->npolytests)/ 1000000.0f);

	}

	if(verbose_stats)
	{
		// Bottom Left
		imgbuf->SetPrintXY(10,  imgbuf->GetHeight() - (90+45), linespace);

		imgbuf->Printlnf("Frustum Dim      %3i", scene->frustumdim);
		imgbuf->Printlnf("Frustum Steps    %3i", scene->frustumstep);
		imgbuf->Printlnf("Frustum Stepsize %3i", scene->frustumstepsize);
		imgbuf->Printlnf("Threads          %3i", args.nthreads);
		imgbuf->Printlnf("RayTraceJobs     %3i", args.njobs);
		imgbuf->SetPrintXY(10,  imgbuf->GetHeight() - 45, linespace);

		imgbuf->Printlnf("Polygons       %5i", scene->numpolys);
	
		// Bottom Right
		imgbuf->SetPrintXY(imgbuf->GetWidth()-160,  imgbuf->GetHeight() - 90, linespace);
		imgbuf->Printlnf("Raster         %3s", scene->raster ? "On" : "Off" );
		imgbuf->Printlnf("Secondary rays %3s", scene->secondrays ? "On" : "Off" );
		imgbuf->Printlnf("Shadow rays    %3s", scene->shadowrays ? "On" : "Off" );
		imgbuf->Printlnf("Shading        %3s", scene->shading ? "On" : "Off" );
		imgbuf->Printlnf("Max recursion  %3i", scene->maxraydepth);
	}
}


int main(int argc, char *argv[])
{
	TestStructs();
	
	#ifdef _CELL
		InitPPECallbacks();
		InitSPEs();
	#endif
		

	Init();

	if(parseArgs(argc, argv) != 0) return 1;

	/* Init framebuffer and Scene */
	printf("lzrt %ix%i (" ARCH_STR ")\n", args.width, args.height);

	ImageBuffer *imgbuf = new FrameBuffer(args.width, args.height, args.fullscreen, "lzrt %ix%i (" ARCH_STR ")", args.width, args.height);

	Scene scene(imgbuf, args.animate, args.renderer);
	
	// Execute Lua script which sets up the scene
	lua_State *L = 0;
	L = InitLua();
	
	if(lua_dofile(L, args.luascript) != 0)
	{
		printf("Error: Couldn't execute Lua script '%s'\n", args.luascript);
		lua_close(L);
		return 1;
	}
	
	threadpool = new ThreadPool(args.nthreads);

	// Setup Scene
	scene.Setup(threadpool);
	
	scene.imgbuf->SetClearColor(0.2f, 0.2f, 0.3f);	
	//scene.imgbuf->SetClearColor(1.0f, 1.0f, 1.0f);

	Timer timer;


	while(run)
	{
		SDLevent();

		threadpool->SetNumThreads(args.nthreads);

		timer.Mark();

		// Call Lua loop function
		if(L != 0 && scene.animate) call_function<void>(L, "lzrtloop");
		
		scene.camera->Move(firstframe, camerapos.x, camerapos.y, camerapos.z);
		scene.imgbuf->Clear();

		scene.Render(threadpool, args.njobs);
		args.njobs = scene.njobs;

		rendertime = timer.GetElapsed();

		PrintStats(imgbuf, &scene);
	
		if(scene.animate || firstframe) frame++;

		imgbuf->Update();

		firstframe = false;
		
	}
}
