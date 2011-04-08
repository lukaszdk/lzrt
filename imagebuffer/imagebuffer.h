#ifndef _IMAGEBUFFER_H_
#define _IMAGEBUFFER_H_

#include <SDL.h>

class ImageBuffer
{
	public:
		float clear_color[3];
		int x,y;
		int linespace;
	public:
		virtual int GetWidth() = 0;
		virtual int GetHeight() = 0;
		virtual void PutPixel(int x, int y, float r, float g, float b) = 0;;
		virtual void Update() = 0;
		virtual void Clear() = 0;
		virtual void Print(int x, int y, char *text) = 0;
		void Printf(int x, int y, char *format, ...);
		void SetPrintXY(int x, int y, int linespace);
		void Printlnf(const char *format, ...);
		void SetClearColor(float r, float g, float b)
		{
			clear_color[0] = r;
			clear_color[1] = g;
			clear_color[2] = b;
		}

};



#endif 


