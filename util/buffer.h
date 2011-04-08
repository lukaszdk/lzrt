#ifndef _BUFFER_H_
#define _BUFFER_H_

#ifdef _CELL
#include <malloc_align.h>
#include <free_align.h>
#endif

template <class E>
class Buffer
{
	public:
		E *buffer;
		int size;
		int index;
	public:
		Buffer()
		{	
			buffer = 0;
			size = 0;
			index = 0;
		}

		~Buffer()
		{
			if(size > 0) 
			#ifdef _CELL
				_free_align(buffer);
			#else
				delete [] buffer;
			#endif
		}		

		void Create(int size)
		{
			if(size == this->size)
			{
				index = 0;
				return;
			}

			if(this->size > 0)
			{
				#ifdef _CELL
					_free_align(buffer);
				#else
					delete [] buffer;
				#endif
			}
	
			#ifdef _CELL			
				buffer = (E*)_malloc_align(sizeof(E) * size, 7);
			#else
				buffer = new E[size];
			#endif
	
			this->size = size;
			index = 0;
		}

		void CopyTo(E *e)
		{
			memcpy(&buffer[index], e, sizeof(E));
			index += 1;
		}

		void CopyTo(Buffer<E> *b)
		{
			memcpy(&buffer[index], b->buffer, b->size*sizeof(E));
			index += b->size;
		}

		E* CopyTo(E *b, int size)
		{
			memcpy(&buffer[index], b, size*sizeof(E));

			E *ret = &buffer[index];
			index += size;

			return ret;
		}

		void* CopyToPtr()
		{
			void* ptr = &buffer[index];
			return ptr;
		}
		
		void Increment(int size)
		{
			index += size;
		}

		void* CopyToPtr(int size)
		{
			void* ptr = &buffer[index];
			index += size;
			return ptr;
		}

		E* Allocate(int size)
		{
			E* ret = &buffer[index];
			index += size;

			return ret;
		}


		E* GetBuffer()
		{
			return buffer;
		}

		void GetBuffer(E **buffer, int *size)
		{
			*buffer = this->buffer;
			*size = this->index;
		}

		int Capacity()
		{
			return size;
		}

		int NumElements()
		{
			return index;
		}

		void Clear()
		{
			index = 0;
		}

		bool IsEmpty()
		{
			return (index == 0);
		}
};

#endif

