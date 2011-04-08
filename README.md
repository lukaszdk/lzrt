About
================================================================================

lzrt is a multiplatform ray tracing application, optimized especially for the 
Cell processor. It has been tested on Ubuntu 8.10 i686, Ubuntu 8.10 Cell 
(PowerPC/SPE) / PS3.

More information available at my [website](http://lukasz.dk/software/lzrt/)

The sourcecode is mainly C++ and compiles with g++ 4.3. The SPE kernels are 
coded in C and use spu-gcc from the Cell SDK to compile.

The following libraries are required to build lzrt: luabind, sdl, sdl-ttf2.0
On Ubuntu you can install the development libraries using the following command.

	sudo apt-get install libluabind-dev libsdl1.2-dev libsdl-ttf2.0-dev

Building i686/PowerPC version
================================================================================

To build the regular/portable lzrt version on either i686 and PowerPC use 
the makefile on either the i686 or PowerPc platform 'Makefile' by typing

	make

Building Cell (SPE) on i686
================================================================================

To build the Cell SPE optimized version on i686 ubuntu use the makefile 
'Makefile.cell'. You need to have the Cell SDK 3.x installed. You also need
the lzrt PowerPC headers installed the lzrt source directory as a 'powerpc/' 
directory.

If the Cell SDK 3.x is installed in any other directory in the standard 
/opt/cell/ you need to modify the CELL_DIR variable in 'Makefile.cell'.

	make -f Makefile.cell


Running
================================================================================

To run lzrt on a lua scene file named 'test.lua' use:

	./lzrt test.lua

Run lzrt without any arguments to see the possible parameters.

