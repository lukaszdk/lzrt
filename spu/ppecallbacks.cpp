#include <malloc_align.h>
#include <libspe2.h>
#include <stdio.h>
#include <spu/kernels/ppecallbacks.h>
#include <semaphore.h>

typedef unsigned int spe_offset_t;

inline void* get_spu_params(void *ls_base_tmp, unsigned int ls_address)
{
	char *ls_base = (char *)ls_base_tmp; 
	spe_offset_t params_offset = *((spe_offset_t *)(ls_base + ls_address));

	return (void*)(ls_base + params_offset);
}


int ppe_sema_post(void *ls_base, unsigned int ls_address) 
{
	ppe_sema_post_params_t *params  = (ppe_sema_post_params_t *)get_spu_params(ls_base, ls_address);
	sem_post((sem_t*)params->sema);
	return 0;
}




void InitPPECallbacks()
{
	spe_callback_handler_register((void*)ppe_sema_post, 0x10, SPE_CALLBACK_NEW);

}


