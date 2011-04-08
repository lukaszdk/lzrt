#include <imagebuffer/imagebuffer.h>


void ImageBuffer::Printf(int x, int y, char *format, ...)
{
	char buffer[1024];

	va_list args;
	va_start (args, format);
	vsprintf (buffer,format, args);
	va_end (args);

	Print(x,y, buffer);
}



void ImageBuffer::SetPrintXY(int x, int y, int linespace)
{
	this->x = x;
	this->y = y;
	this->linespace = linespace;
}
		

void ImageBuffer::Printlnf(const char *format, ...)
{
	char buffer[1024];

	va_list args;
	va_start (args, format);
	vsprintf (buffer,format, args);
	va_end (args);

	Print(x,y, buffer);

	y += linespace;

}


