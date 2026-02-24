all: ddsli.oct links

ddsli.oct: ddsli_core.c ddsli_core.h ddsli_oct.cc
	mkoctfile -o ddsli ddsli_core.c ddsli_oct.cc

links:
	ln -sf ddsli.oct ddsli_open.oct
	ln -sf ddsli.oct ddsli_send_cmd.oct
	ln -sf ddsli.oct ddsli_toggle_out_stream.oct
	ln -sf ddsli.oct ddsli_read_blocks.oct
	ln -sf ddsli.oct ddsli_read_adc.oct
	ln -sf ddsli.oct ddsli_close.oct
	ln -sf ddsli.oct ddsli_read_blocks_matrix.oct
	ln -sf ddsli.oct ddsli_read_adc_matrix.oct