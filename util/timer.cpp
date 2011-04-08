#include <util/timer.h>


Timer::Timer()
{
	Reset();
}


void Timer::Reset()
{
	Uint32 time = SDL_GetTicks();

	resettime = time;
	elaptime = time;
}

Uint32 Timer::GetTime()
{
	return SDL_GetTicks() - resettime;
}

void Timer::Mark()
{
	elaptime = SDL_GetTicks();
}

Uint32 Timer::GetElapsed(bool mark)
{
	Uint32 time = SDL_GetTicks();

	Uint32 rtime = time - elaptime;
	
	if(mark) elaptime = time;
	
	return rtime;
}


