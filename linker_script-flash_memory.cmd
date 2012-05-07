/* ****************************************************************************************************** */
/*   linker_script-flash_memory.cmd				LINKER  SCRIPT                                                */
/*                                                                                                        */
/*                                                                                                        */
/*   The Linker Script defines how the code and data emitted by the GNU C compiler and assembler are  	  */
/*   to be loaded into memory (code goes into FLASH, variables go into RAM).                 			  */
/*                                                                                                        */
/*   Any symbols defined in the Linker Script are automatically global and available to the rest of the   */
/*   program.                                                                                             */
/*                                                                                                        */
/*   To force the linker to use this LINKER SCRIPT, just add the -T demo2106_blink_flash.cmd directive    */
/*   to the linker flags in the makefile.                                                                 */
/*                                                                                                        */
/*   			LFLAGS  =  -Map main.map -nostartfiles -T linker_script-flash_memory.cmd                        */
/*                                                                                                        */
/*                                                                                                        */
/*   The Philips boot loader supports the ISP (In System Programming) via the serial port and the IAP     */
/*   (In Application Programming) for flash programming from within your application.                     */
/*                                                                                                        */
/*   The boot loader uses RAM memory and we MUST NOT load variables or code in these areas.               */
/*                                                                                                        */
/*   RAM used by boot loader:  0x40000120 - 0x400001FF  (223 bytes) for ISP variables                     */
/*                             0x4000FFE0 - 0x4000FFFF  (32 bytes)  for ISP and IAP variables             */
/*                             0x4000FEE0 - 0x4000FFDF  (256 bytes) stack for ISP and IAP                 */
/*                                                                                                        */
/*  Original file by:  James P. Lynch. Modified by Jorge "Casainho" Pinto on April.2009. */
/*                                                                                                        */
/* ****************************************************************************************************** */

/* identify the Entry Point  */
ENTRY(_startup)

/* specify the LPC2103 memory areas  */
MEMORY 
{
    flash           : ORIGIN = 0,          LENGTH = (32K - 4K)  /* FLASH ROM                                */  
    ram_isp_low(A)      : ORIGIN = 0x40000040, LENGTH = 224 /* variables used by Philips ISP bootloader */       
    ram             : ORIGIN = 0x40000120, LENGTH = 7872    /* free RAM area                */
    ram_isp_high(A)     : ORIGIN = 0x40001FE0, LENGTH = 32  /* variables used by Philips ISP bootloader */
}

/* define a global symbol _stack_end  */
_stack_end = 0x40001FDC;

/* now define the output sections  */
SECTIONS 
{
	. = 0;								/* set location counter to address zero  */
	
	startup :
	{
	       *(.startup)
    } >flash		/* the startup code goes into FLASH */
	
	.text :						/* collect all sections that should go into FLASH after startup  */ 
	{
		*(.text)						/* all .text sections (code)  */
		*(.rodata)						/* all .rodata sections (constants, strings, etc.)  */
		*(.rodata*)						/* all .rodata* sections (constants, strings, etc.)  */
		*(.glue_7)						/* all .glue_7 sections  (no idea what these are) */
		*(.glue_7t)						/* all .glue_7t sections (no idea what these are) */
		_etext = .;				/* define a global symbol _etext just after the last code byte */
	} >flash							/* put all the above into FLASH */

	.data :								/* collect all initialized .data sections that go into RAM  */ 
	{
		_data = .;			/* create a global symbol marking the start of the .data section  */
		*(.data)						/* all .data sections  */
		_edata = .;	      /* define a global symbol marking the end of the .data section  */
	} >ram AT >flash /* put all the above into RAM (but load the LMA copy into FLASH) */

	.bss :								/* collect all uninitialized .bss sections that go into RAM  */
	{
		_bss_start = .;		/* define a global symbol marking the start of the .bss section */
		*(.bss)						/* all .bss sections  */
	} >ram				/* put all the above in RAM (it will be cleared in the startup code */

	. = ALIGN(4);					/* advance location counter to the next 32-bit boundary */
	_bss_end = . ;			/* define a global symbol marking the end of the .bss section */
}
	_end = .;						/* define a global symbol marking the end of application RAM */
