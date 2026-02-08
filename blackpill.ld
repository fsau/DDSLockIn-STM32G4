MEMORY
{
    rom (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 22K
    ccm (rwx) : ORIGIN = 0x10000000, LENGTH = 10K
}

/* Enforce emmition of the vector table. */
EXTERN (vector_table)

/* Define the entry point of the output file. */
ENTRY(reset_handler)

/* Define sections. */
SECTIONS
{
	.text : {
		KEEP(*(.vectors)) /* Vector table */
		*(.text*)	/* Program code */
		. = ALIGN(4);
		*(.rodata*)	/* Read-only data */
		. = ALIGN(4);
	} >rom

	/* C++ Static constructors/destructors, also used for __attribute__
	 * ((constructor)) and the likes */
	.preinit_array : {
		. = ALIGN(4);
		__preinit_array_start = .;
		KEEP (*(.preinit_array))
		__preinit_array_end = .;
	} >rom
	.init_array : {
		. = ALIGN(4);
		__init_array_start = .;
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array))
		__init_array_end = .;
	} >rom
	.fini_array : {
		. = ALIGN(4);
		__fini_array_start = .;
		KEEP (*(.fini_array))
		KEEP (*(SORT(.fini_array.*)))
		__fini_array_end = .;
	} >rom

	/*
	 * Another section used by C++ stuff, appears when using newlib with
	 * 64bit (long long) printf support
	 */
	.ARM.extab : {
		*(.ARM.extab*)
	} >rom
	.ARM.exidx : {
		__exidx_start = .;
		*(.ARM.exidx*)
		__exidx_end = .;
	} >rom

	. = ALIGN(4);
	_etext = .;

	/* ram, but not cleared on reset, eg boot/app comms */
	.noinit (NOLOAD) : {
		*(.noinit*)
	} >ram
	. = ALIGN(4);

	.data : {
		_data = .;
		*(.data*)	/* Read-write initialized data */
		*(.ramtext*)    /* "text" functions to run in ram */
        *(.code_in_ram)
		. = ALIGN(4);
		_edata = .;
	} >ram AT >rom
	_data_loadaddr = LOADADDR(.data);

	.bss : {
		*(.bss*)	/* Read-write zero initialized data */
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
	} >ram

	/* --- CCM text (executable code) --- */
    .ccm_text :
    {
        . = ALIGN(4);
        _sccm_text = .;
        *(.ccm_text*)
        . = ALIGN(4);
        _eccm_text = .;
    } > ccm AT > rom
    _ccm_text_loadaddr = LOADADDR(.ccm_text);
    
    /* --- CCM data (initialized) --- */
    .ccm_data :
    {
        . = ALIGN(4);
        _sccm_data = .;
        *(.ccm_data*)
        . = ALIGN(4);
        _eccm_data = .;
    } > ccm AT > rom
    _ccm_data_loadaddr = LOADADDR(.ccm_data);
    
    /* --- CCM BSS (uninitialized) --- */
    .ccm_bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sccm_bss = .;
        *(.ccm_bss*)
        . = ALIGN(4);
        _eccm_bss = .;
    } > ccm
    
    /* --- CCM noinit (not zeroed) --- */
    .ccm_noinit (NOLOAD) :
    {
        . = ALIGN(4);
        *(.ccm_noinit*)
        . = ALIGN(4);
    } > ccm

	/*
	 * The .eh_frame section appears to be used for C++ exception handling.
	 * You may need to fix this if you're using C++.
	 */
	/DISCARD/ : { *(.eh_frame) }

	. = ALIGN(4);
	end = .;
}

/* Stack at top of CCM */
PROVIDE(_stack = ORIGIN(ccm) + LENGTH(ccm));
