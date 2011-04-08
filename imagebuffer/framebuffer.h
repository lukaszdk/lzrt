#ifndef _FRAMEBUFFER_H_
#define _FRAMEBUFFER_H_

#include <imagebuffer/imagebuffer.h>
#include "SDL.h"
#include "SDL_ttf.h"

class FrameBuffer : public ImageBuffer
{
	public:
		int width;
		int height;
		SDL_Surface *surface;
		TTF_Font *font;
		Uint32 *pixels;
		Uint32 *offscreen;
	public:
		FrameBuffer(int width, int height, bool fullscreen, const char *format, ...);
		~FrameBuffer();
		int GetWidth();
		int GetHeight();
		void PutPixel(int x, int y, float r, float g, float b);
		void Update();
		void Clear();
		void Print(int x, int y, char *text);	
		void SetPrintXY(int x, int y, int linespace);

};

#endif


