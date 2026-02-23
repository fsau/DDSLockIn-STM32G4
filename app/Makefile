all: ddsli_open.oct

ddsli_open.oct: ddsli_core.c ddsli_core.h ddsli_mex.cc
	mkoctfile -o ddsli_open ddsli_core.c ddsli_mex.cc