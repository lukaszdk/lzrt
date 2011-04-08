OBJS  = lwob.o lwio.o list.o surface.o pntspols.o vecmath.o clip.o 
OBJS += envelope.o lwoloader.o lwo2.o vmap.o

BIN_OBJS = main.o $(OBJS) ../lzrt/server/aabb.o ../lzrt/server/kdtree.o ../lzrt/server/stats.o

BIN = lwoloader

LIB := liblwoloader.a


CPPFLAGS := -I. -I../lzmath -I/usr/include/ $(shell sdl-config --cflags)
LDFLAGS := $(shell sdl-config --libs) -L../lzmath -lGL -lGLU -llzmath

all:  $(LIB)

%.o: %.cpp
	g++ -c $(CPPFLAGS) $< -o $@

%.o: %.c
	gcc -c $(CPPFLAGS) $< -o $@

$(BIN): $(BIN_OBJS)
	g++ $(BIN_OBJS) -o $(BIN) $(LDFLAGS)

$(LIB): $(OBJS)
	$(AR) rcu $(LIB) $(OBJS)

clean:
	rm -f $(BIN) $(OBJS) $(LIB)

