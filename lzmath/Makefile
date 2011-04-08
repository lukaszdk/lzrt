
OBJS := lzvector.o lzmatrix.o lztransform.o lzpoint.o lzray.o lznormal.o
LIB := liblzmath.a

TEST_OBJS := Test.o
TEST_BIN := Test

CC := g++
AR := ar

CFLAGS := -ansi -Wall
LDFLAGS := -L. -llzmath
INCS := -I.

all: $(LIB)

ppu:
	make -f Makefile.ppu

%.o: %.cpp
	$(CC) $(INCS) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(TEST_OBJS) $(OBJS) $(LIB) ${TEST_BIN}


$(LIB): $(OBJS)
	$(AR) rcu $(LIB) $(OBJS)

$(TEST_BIN): $(TEST_OBJS)
	$(CC) $(TEST_OBJS) -o $(TEST_BIN) $(LDFLAGS) 


run: $(TEST_BIN)
	./$(TEST_BIN)

