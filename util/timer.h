#ifndef _TIMER_H_
#define _TIMER_H_

#include <SDL.h>

class Timer
{
	public:
		Uint32 resettime;
		Uint32 elaptime;
	public:
		Timer();
		void Reset();
		void Mark();
		Uint32 GetTime();
		Uint32 GetElapsed(bool mark = true);
};



#endif

