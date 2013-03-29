/*
Default linker script for STM32F10x_128K_8K with GNU-ld
Martin Thomas, 9/2009
Jorge Pinto, 1/2013
A heavily modified "Default linker script for STM32F10x_512K_64K" 
Copyright RAISONANCE S.A.S. 2008 found in the STM fw-library.
(there is not much left from Raisonance's code and nearly everything
has been modified or reordered)
*/

/* Program Entry, set to mark it as "used" and avoid gc */
ENTRY(Reset_Handler)

/* Memory Spaces Definitions */
MEMORY
{
	RAM      (RWX) : ORIGIN = 0x20000000+0, LENGTH = 4K-0
	EXTSRAM  (RWX) : ORIGIN = 0x68000000, LENGTH = 0
	FLASH    (RX)  : ORIGIN = 0x08000000+0, LENGTH = 128K-2K-0
	FLASHB1  (RX)  : ORIGIN = 0x00000000, LENGTH = 0
	EXTMEMB0 (RX)  : ORIGIN = 0x00000000, LENGTH = 0
	EXTMEMB1 (RX)  : ORIGIN = 0x00000000, LENGTH = 0
	EXTMEMB2 (RX)  : ORIGIN = 0x00000000, LENGTH = 0
	EXTMEMB3 (RX)  : ORIGIN = 0x00000000, LENGTH = 0
}

/* higher address of the user mode stack */
_estack = ORIGIN(RAM)+LENGTH(RAM);

/* start of the two 1kByte pages used by EEprom emulation at end of flash */
/*_seemul = ORIGIN(EEMUL);*/

/* There will be a link error if there is not this amount of RAM free at the end. */
_Minimum_Stack_Size = 0x100 ;

/* Check valid alignment for VTOR */
ASSERT(ORIGIN(FLASH) == ALIGN(ORIGIN(FLASH), 0x80), "Start of memory region flash not aligned for startup vector table");

/* Sections Definitions */

SECTIONS
{

	/* the program code is stored in the .text section, which goes to Flash */
	.text :
	{
		. = ALIGN(0x80); /* PM0056, Rev.1 (4/2009), 4.3.3 */
		_isr_vectorsflash_offs = . - 0x08000000;
		KEEP(*(.isr_vectorsflash))
		. = ALIGN(4);
		CREATE_OBJECT_SYMBOLS
		*(.text .text.* .gnu.linkonce.t.*)
		*(.plt)
		*(.gnu.warning)
		*(.glue_7t) *(.glue_7) *(.vfp11_veneer)
		*(.ARM.extab* .gnu.linkonce.armextab.*)
		*(.gcc_except_table)
	} >FLASH

	.eh_frame_hdr : ALIGN (4)
	{
		KEEP (*(.eh_frame_hdr))
	} >FLASH
	.eh_frame : ALIGN (4)
	{
		KEEP (*(.eh_frame))
	} >FLASH
	
	/* .ARM.exidx is sorted, so has to go in its own output section.  */
	__exidx_start = .;
	.ARM.exidx :
	{
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} >FLASH
	__exidx_end = .;

	.rodata : ALIGN (4)
	{
		*(.rodata .rodata.* .gnu.linkonce.r.*)
		
		. = ALIGN(4);
		KEEP(*(.init))
		
		. = ALIGN(4);
		__preinit_array_start = .;
		KEEP (*(.preinit_array))
		__preinit_array_end = .;
		
		. = ALIGN(4);
		__init_array_start = .;
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array))
		__init_array_end = .;
		
		. = ALIGN(4);
		KEEP(*(.fini))
		
		. = ALIGN(4);
		__fini_array_start = .;
		KEEP (*(.fini_array))
		KEEP (*(SORT(.fini_array.*)))
		__fini_array_end = .;
		
		. = ALIGN(0x4);
		KEEP (*crtbegin.o(.ctors))
		KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
		KEEP (*(SORT(.ctors.*)))
		KEEP (*crtend.o(.ctors))
		
		. = ALIGN(0x4);
		KEEP (*crtbegin.o(.dtors))
		KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
		KEEP (*(SORT(.dtors.*)))
		KEEP (*crtend.o(.dtors))
		
		*(.init .init.*)
		*(.fini .fini.*)
		
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP (*(.preinit_array))
		PROVIDE_HIDDEN (__preinit_array_end = .);
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array))
		PROVIDE_HIDDEN (__init_array_end = .);
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP (*(.fini_array))
		KEEP (*(SORT(.fini_array.*)))
		PROVIDE_HIDDEN (__fini_array_end = .);
		
		. = ALIGN (8);
		*(.rom)
		*(.rom.b)
		_etext = .;
		/* This is used by the startup in order to initialize the .data secion */
		_sidata = _etext;
	} >FLASH

	/* This is the initialized data section
	The program executes knowing that the data is in the RAM
	but the loader puts the initial values in the FLASH (inidata).
	It is one task of the startup to copy the initial values from FLASH to RAM. */
	.data : ALIGN (8)
	{
		/* This is used by the startup in order to initialize the .data section */
		_sdata = . ;
		
		. = ALIGN(0x80);
		_isr_vectorsram_offs = . - 0x20000000;
		KEEP(*(.isr_vectorsram))
		
		. = ALIGN(4);
		KEEP(*(.jcr))
		*(.got.plt) *(.got)
		*(.shdata)
		*(.data .data.* .gnu.linkonce.d.*)
		. = ALIGN (8);
		*(.ram)
		*(.ramfunc*)
		 . = ALIGN(4);
		/* This is used by the startup in order to initialize the .data section */
		_edata = .;
	} >RAM AT>FLASH

	/* This is the uninitialized data section */
	.bss (NOLOAD):
	{
		. = ALIGN(4);
		/* This is used by the startup in order to initialize the .bss section */
		_sbss = .;
		*(.shbss)
		*(.bss .bss.* .gnu.linkonce.b.*)
		*(COMMON)
		. = ALIGN (8);
		*(.ram.b)
		. = ALIGN(4);
		 /* This is used by the startup in order to initialize the .bss section */
		 _ebss = . ;
		_end = .;
		__end = .;
	} >RAM AT>FLASH


    /* This is the user stack section 
    This is just to check that there is enough RAM left for the User mode stack
    It should generate an error if it's full.
     */
    ._usrstack (NOLOAD):
    {
        . = ALIGN(4);
        _susrstack = . ;
        . = . + _Minimum_Stack_Size ;
        . = ALIGN(4);
        _eusrstack = . ;
    } >RAM


    /* this is the FLASH Bank1 */
    /* the C or assembly source must explicitly place the code or data there
    using the "section" attribute */
    .b1text :
    {
        *(.b1text)                   /* remaining code */
        *(.b1rodata)                 /* read-only data (constants) */
        *(.b1rodata*)
    } >FLASHB1
    
    /* this is the EXTMEM */
    /* the C or assembly source must explicitly place the code or data there
    using the "section" attribute */
    
    /* EXTMEM Bank0 */
    .eb0text :
    {
        *(.eb0text)                   /* remaining code */
        *(.eb0rodata)                 /* read-only data (constants) */
        *(.eb0rodata*)
    } >EXTMEMB0
    
    /* EXTMEM Bank1 */
    .eb1text :
    {
        *(.eb1text)                   /* remaining code */
        *(.eb1rodata)                 /* read-only data (constants) */
        *(.eb1rodata*)
    } >EXTMEMB1
    
    /* EXTMEM Bank2 */
    .eb2text :
    {
        *(.eb2text)                   /* remaining code */
        *(.eb2rodata)                 /* read-only data (constants) */
        *(.eb2rodata*)
    } >EXTMEMB2
    
    /* EXTMEM Bank0 */
    .eb3text :
    {
        *(.eb3text)                   /* remaining code */
        *(.eb3rodata)                 /* read-only data (constants) */
        *(.eb3rodata*)
    } >EXTMEMB3
    
    
    
    /* after that it's only debugging information. */
    
    /* remove the debugging information from the standard libraries */
/*    DISCARD :
    {
     libc.a ( * )
     libm.a ( * )
     libgcc.a ( * )
     }
*/

  /* Stabs debugging sections.  */
  .stab          0 : { *(.stab) }
  .stabstr       0 : { *(.stabstr) }
  .stab.excl     0 : { *(.stab.excl) }
  .stab.exclstr  0 : { *(.stab.exclstr) }
  .stab.index    0 : { *(.stab.index) }
  .stab.indexstr 0 : { *(.stab.indexstr) }
  .comment       0 : { *(.comment) }
  /* DWARF debug sections.
     Symbols in the DWARF debugging sections are relative to the beginning
     of the section so we begin them at 0.  */
  /* DWARF 1 */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }
  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames  0 : { *(.debug_varnames) }
  /* DWARF 3 */
  .debug_pubtypes 0 : { *(.debug_pubtypes) }
  .debug_ranges   0 : { *(.debug_ranges) }

  .ARM.attributes 0 : { KEEP (*(.ARM.attributes)) KEEP (*(.gnu.attributes)) }
  .note.gnu.arm.ident 0 : { KEEP (*(.note.gnu.arm.ident)) }
  /DISCARD/ : { *(.note.GNU-stack) *(.gnu_debuglink) }
}

/* mthomas:  from the STM example - moved here for now to keep it as reference, not 
   used in the project (useful anyway?)*/

/* default stack size(s). These are used by the startup in order to allocate stacks for 
the different modes. */
__Stack_Size = 1024 ;

PROVIDE ( _Stack_Size = __Stack_Size ) ;
__Stack_Init = _estack - __Stack_Size ;

/*"PROVIDE" allows to easily override these values from an object file or the commmand line.*/
PROVIDE ( _Stack_Init = __Stack_Init ) ;
