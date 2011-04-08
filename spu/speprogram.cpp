#include <spu/speprogram.h>


static int numspes = 0;
static SPEContext *context;

void InitSPEs()
{
	numspes = spe_cpu_info_get(SPE_COUNT_PHYSICAL_SPES, 0);
	context = new SPEContext[numspes];
		
	//cout << "numSPEs : " << numspes << endl;

}

int NumSPEs()
{
	return numspes;
}

SPEContext *GetSPEContext(int spe)
{
	//cout << "spe " << spe << endl;

	if(numspes > 0 && spe < numspes)
	{
		return &context[spe];
	}

	return NULL;
}

