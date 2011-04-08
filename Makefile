
BIN = lzrt

OBJS := main.o imagebuffer/framebuffer.o imagebuffer/imagebuffer.o
OBJS += mesh/aabb.o mesh/custommesh.o mesh/lwomesh.o
OBJS += scene/camera.o scene/kdbuild.o scene/kdbuildjob.o scene/kdtravers.o  
OBJS += scene/meshtransform.o scene/meshtransformjob.o scene/polytest.o scene/raygen.o scene/raytracejob.o 
OBJS += scene/scene.o util/luabind.o util/threadpool.o util/timer.o

INCLUDES := -I. -Ilzmath -Ilwoloader -I/usr/include/lua50 -I/usr/include/boost
CPPFLAGS :=  -g $(INCLUDES) $(shell sdl-config --cflags) 
LIBS     := -llzmath -llwoloader -lSDL_ttf  -llua50 -llualib50 -lluabind -lpthread
LDFLAGS  := -g -Llzmath -Llwoloader $(LIBS) $(shell sdl-config --libs)

all: $(BIN) 

lzmath/liblzmath.a:
	make -C lzmath all

lwoloader/liblwoloader.a:
	make -C lwoloader all

%.o: %.cpp
	g++ -c $(CPPFLAGS) $< -o $@

$(BIN): $(OBJS) lzmath/liblzmath.a lwoloader/liblwoloader.a
	g++ $(OBJS) -o $(BIN) $(LDFLAGS)

clean:
	rm -f $(BIN) $(OBJS)


clean-all:
	rm -f $(BIN) $(OBJS)
	make -C lzmath clean
	make -C lwoloader clean

