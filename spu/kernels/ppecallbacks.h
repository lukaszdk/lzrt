#ifndef _SPU_PPECALLBACKS_H_
#define _SPU_PPECALLBACKS_H_


#ifdef __SPU__
#include <share/structs.h>
#include <sys/syscall.h>
#include <stdio.h>
#endif

typedef struct
{
	void *sema;
} ppe_sema_post_params_t;

#ifdef __SPU__

void ppe_post_sema(void *sema)
{
	ppe_sema_post_params_t params ALIGNED(16) = { sema };
	__send_to_ppe(0x2110, 0, &params);
}


#endif

#endif



