PYTHON_EXT_SUFFIX = $(shell python3-config --extension-suffix)
LINK_TARGETS = ddsli_open.oct ddsli_send_cmd.oct ddsli_toggle_out_stream.oct \
               ddsli_read_blocks.oct ddsli_read_adc.oct ddsli_close.oct \
               ddsli_read_blocks_matrix.oct ddsli_read_adc_matrix.oct

all: ddsli.oct ddsli$(PYTHON_EXT_SUFFIX) $(LINK_TARGETS)

ddsli.oct: ddsli_core.c ddsli_core.h ddsli_oct.cc
	mkoctfile -o ddsli ddsli_core.c ddsli_oct.cc

ddsli$(PYTHON_EXT_SUFFIX): ddsli_core.o ddsli_module.cc
	gcc -O2 -fPIC $(shell python3-config --includes) -shared ddsli_module.cc -o ddsli$(PYTHON_EXT_SUFFIX) -lstdc++ ddsli_core.o

ddsli_core.o: ddsli_core.c ddsli_core.h
	gcc -O3 -c ddsli_core.c -o ddsli_core.o

$(LINK_TARGETS): %: ddsli.oct
	ln -sf $< $@

clean:
	rm -f ddsli.oct ddsli$(PYTHON_EXT_SUFFIX) ddsli_core.o $(LINK_TARGETS)