#include <imagebuffer/framebuffer.h>

#define DEFAULT_PTSIZE 18

static void quit(int rc)
{
	SDL_Quit();
	exit(rc);
}

FrameBuffer::FrameBuffer(int width, int height, bool fullscreen, const char *format, ...)
{
	char caption[1024];

	va_list args;
	va_start (args, format);
	vsprintf (caption,format, args);
	va_end (args);

	this->width = width;
	this->height = height;

	/* Initialize SDL */
	if ( SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0 ) {
		fprintf(stderr, "Couldn't initialize SDL: %s\n",SDL_GetError());
		
		quit(1);
	}

	if ( TTF_Init() < 0 ) 
	{
		fprintf(stderr, "Couldn't initialize TTF: %s\n",SDL_GetError());
		SDL_Quit();

		quit(2);
	}

	font = TTF_OpenFont("font/VeraMono.ttf", 13);

	if ( font == NULL ) 
	{
		fprintf(stderr, "Couldn't load %d pt font from %s: %s\n",
					DEFAULT_PTSIZE,"VeraMono.ttf", SDL_GetError());
	}

	int flags = SDL_HWSURFACE | SDL_ANYFORMAT | SDL_DOUBLEBUF;

	if(fullscreen) flags |= SDL_FULLSCREEN;


	surface = SDL_SetVideoMode(width, height, 8, flags);

	// printf("SDL_GetError(): '%s'\n", SDL_GetError());

	if ( ! surface ) {
		fprintf(stderr, "Couldn't set %dx%d video mode: %s\n",
					width, height, SDL_GetError());
		quit(2);
	}

	SDL_WM_SetCaption(caption, 0);

	atexit(SDL_Quit);

}

FrameBuffer::~FrameBuffer()
{
	SDL_FreeSurface(surface);
}


int FrameBuffer::GetWidth()
{
	return width;
}

int FrameBuffer::GetHeight()
{
	return height;
}

void FrameBuffer::PutPixel(int x, int y, float r, float g, float b)
{
	// Clamp;
	/*
	if(r < 0) r = 0;
	if(r > 1) r = 1;
	if(g < 0) g = 0;
	if(g > 1) g = 1;
	if(b < 0) b = 0;
	if(b > 1) b = 1;
	*/

	Uint32 c = 0;
	Uint8 *bc = (Uint8*)&c;

	#ifndef _CELL
		bc[2] = (Uint8)(r * 0xFF);
		bc[1] = (Uint8)(g * 0xFF);
		bc[0] = (Uint8)(b * 0xFF);
	#else
		bc[1] = (Uint8)(r * 0xFF);
		bc[2] = (Uint8)(g * 0xFF);
		bc[3] = (Uint8)(b * 0xFF);
	#endif
	

	/*
	Uint32 ri = (int)(r * 0xFF);
	Uint32 gi = (int)(g * 0xFF);
	Uint32 bi = (int)(b * 0xFF);
	*/

	Uint32 *pixels = (Uint32*)surface->pixels;

	pixels[(y*width)+x] = c;

	//pixels[(y*width)+x] = (ri << 16) | (gi << 8) | bi;
}

void FrameBuffer::Update()
{
	SDL_Flip(surface);
}

void FrameBuffer::Clear()
{
	float r = clear_color[0];
	float g = clear_color[1];
	float b = clear_color[2];

	if(r < 0) r = 0;
	if(r > 1) r = 1;
	if(g < 0) g = 0;
	if(g > 1) g = 1;
	if(b < 0) b = 0;
	if(b > 1) b = 1;

	Uint32 ri = (int)(r * 0xFF);
	Uint32 gi = (int)(g * 0xFF);
	Uint32 bi = (int)(b * 0xFF);

	Uint32 clearcolor = (ri << 16) | (gi << 8) | bi;

	SDL_FillRect( surface, NULL, clearcolor);
}

void FrameBuffer::Print(int x, int y, char *text)
{
	SDL_Color color = { 0xFF, 0xFF, 0xFF, 0};
	SDL_Surface *sText = TTF_RenderText_Blended( font, text, color );

	SDL_Rect rcDest = { x,y,0,0};

   	SDL_BlitSurface( sText, NULL, surface, &rcDest );
	SDL_FreeSurface(sText);

}







