#ifndef _SPEPROGRAM_H_
#define _SPEPROGRAM_H_

#include <libspe2.h>
#include <errno.h>
#include <stdlib.h>
#include <util/timer.h>
#include <iostream>

using namespace std;

class SPEContext
{
	public:
		spe_context_ptr_t ctx;
	public:
		SPEContext()
		{
			if ((ctx = spe_context_create (0, NULL)) == NULL) 
			{
 	     		perror ("SPEContext: Failed creating context");
    	  		exit (1);
			}

		}

		~SPEContext()
		{
			if (spe_context_destroy (ctx) != 0) 
			{
     	 		perror("SPEContext: Failed destroying context");
      			exit (1);
 	  		}
		}
};


class SPEProgram
{
	public:
		spe_program_handle_t *handle;
	public:
		SPEProgram(spe_program_handle_t *handle)
		{
			this->handle = handle;						
		}

		void Run(SPEContext *spectxt, void *arg = NULL)
		{
			if (spe_program_load (spectxt->ctx, handle)) 
			{
      			perror ("SPEProgram: Failed loading program");
      			exit (1);
			}

			unsigned int entry = SPE_DEFAULT_ENTRY;

			if (spe_context_run(spectxt->ctx, &entry, 0, arg, NULL, NULL) < 0) 
			{
    			perror ("SPEProgram: Failed running context");
   				exit (1);
  			}


		}
};

void InitSPEs();
int NumSPEs();
SPEContext *GetSPEContext(int spe);



#endif

