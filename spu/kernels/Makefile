
KERNELS = meshtransform.o raygen.o polypartition.o polytest.o kdtravers.o kdbuild.o
TIMINGS = meshtransform.t raygen.t polypartition.t polytest.t kdtravers.t kdbuild.t


CELL_T := /opt/cell/
CC := $(CELL_T)toolchain/bin/spu-gcc
EMBEDSPU := $(CELL_T)toolchain/bin/ppu-embedspu

SPU_TIMING := $(CELL_T)sdk/usr/bin/spu_timing

INCLUDE :=  -I../ -I../../ -I.  -I $(CELL_T)/sysroot/usr/spu/include -I $(CELL_T)sysroot/opt/cell/sdk/usr/spu/include -I $(CELL_T)sysroot/opt/cell/sdk/usr/include
CFLAGS := -W -Wall -Winline -Wno-main -O3 -c -fno-strict-aliasing -D_CELL
LDFLAGS :=  -Wl,-N    

all: $(KERNELS)

%.ospe: %.c
	$(CC) $(INCLUDE) -g $(CFLAGS) $< -o $@	

%.bin: %.ospe
	$(CC) $(LDFLAGS)-g $< -o $@

%.o: %.bin
	$(EMBEDSPU) -m32 $(basename $<) $< $@

%.s: %.c
	$(CC) $(INCLUDE) $(CFLAGS) -S $<

%.t: %.s
	$(SPU_TIMING) -o $@ $<

timing: $(TIMINGS)

clean:
	rm -f $(KERNELS) $(TIMINGS)

.SUFFIXES:

