#include "lpc210x.h"
#include "main.h"

void IRQ_Routine (void)
{
    while (1) ;
}

void FIQ_Routine (void)
{
    while (1) ;
}

void SWI_Routine (void)
{
    while (1) ;
}

void UNDEF_Routine (void)
{
    while (1) ;
}

void feed(void)
{
  PLLFEED=0xAA;
  PLLFEED=0x55;
}

#ifdef XTAL_12000000HZ
#define PLOCK 0x400
void system_init (void)
{
    unsigned char i;

    //              Setting the Phased Lock Loop (PLL)
    //               ----------------------------------
    //
    // Zero LPC2103 board has a 12.000 mhz crystal
    //
    // We'd like the LPC2103 to run at 48.000 Mhz (has to be an even multiple of crystal)
    //
    // According to the Philips LPC2103 manual:   M = cclk / Fosc   where:  M    = PLL multiplier (bits 0-4 of PLLCFG)
    //                                                                      cclk = 48000000 hz
    //                                                                      Fosc = 12000000 hz
    //
    // Solving: M = 48000000 / 12000000 = 4
    // M = 4 (round up) --> real cclk =
    //                          = processor clock = 12000000 * 4 = 48000000 Hz.
    //
    //          Note: M - 1 must be entered into bits 0-4 of PLLCFG (assign 3 to these bits)
    //
    //
    // The Current Controlled Oscilator (CCO) must operate in the range 156 mhz to 320 mhz
    //
    // According to the Philips LPC2106 manual: Fcco = cclk * 2 * P    where:   Fcco = CCO frequency
    //                                                                          cclk = 48000000 hz
    //                                                                          P = PLL divisor (bits 5-6 of PLLCFG)
    //
    // Solving: Fcco = 48000000 * 2 * P
    //          P = 2  (trial value)
    //          Fcco = 48000000 * 2 * 2
    //          Fcc0 = 192000000 hz    (good choice for P since it's within the 156 mhz to 320 mhz range
    //
    // From Table 19 (page 48) of Philips LPC2106 manual    P = 2, PLLCFG bits 5-6 = 1  (assign 1 to these bits)
    //
    // Finally:      PLLCFG = 0  01  00011  =  0x23
    //
    // Final note: to load PLLCFG register, we must use the 0xAA followed 0x55 write sequence to the PLLFEED register
    //             this is done in the short function feed() below
    //

    // Setting Multiplier and Divider values
    PLLCFG=0x23;
    feed();

    // Enabling the PLL */
    PLLCON=0x1;
    feed();

    // Wait for the PLL to lock to set frequency
    while(!(PLLSTAT & PLOCK)) ;

    // Connect the PLL as the clock source
    PLLCON=0x3;
    feed();

    // Enabling MAM and setting number of clocks used for Flash memory fetch (4 cclks in this case)
    MAMCR=0x2;
    MAMTIM=0x4;

    // Setting peripheral Clock (pclk) to System Clock (cclk)
    VPBDIV=0x1;

    /* Disable the power for all pheripherials */
    PCONP &= ~((1 << 1) | (1 << 2) | (1 << 4) | (1 << 7) | \
             (1 << 8) | (1 << 9) | (1 << 10) | (1 << 12) | (1 << 19) | \
             (1 << 22) | (1 << 23));

    /* VIC initialization */
    VICINTENCLR = 0xFFFF; /* first disable all interrupts at the VIC */
    VICSOFTINTCLR = 0xFFFF; /* clear soft interrupts */
    VICINTSEL = 0; /* reset all interrupts as IRQ (not FIQ) */

    for(i=0; i<16; i++) /* reset all vectors in the VIC */
    {
        (&VICVECTCNTL0)[i] = 0;
        (&VICVECTADDR0)[i] = 0;
    }

    VICDEFVECTADDR = 0; /* set default handler */
}
#endif

#ifdef XTAL_14745600HZ
#define PLOCK 0x400
void system_init (void)
{
    unsigned char i;

    //              Setting the Phased Lock Loop (PLL)
    //               ----------------------------------
    //
    // Olimex board has a 14.7456 mhz crystal
    //
    // We'd like the LPC2106 to run at 53.2368 Mhz (has to be an even multiple of crystal)
    //
    // According to the Philips LPC2103 manual:   M = cclk / Fosc   where:  M    = PLL multiplier (bits 0-4 of PLLCFG)
    //                                                                      cclk = 53236800 hz
    //                                                                      Fosc = 14745600 hz
    //
    // Solving: M = 53236800 / 14745600 = 3.6103515625
    // M = 4 (round up) --> real cclk =
    //                          = processor clock = 14745600 * 4 = 58982400 Hz.
    //
    //          Note: M - 1 must be entered into bits 0-4 of PLLCFG (assign 3 to these bits)
    //
    //
    // The Current Controlled Oscilator (CCO) must operate in the range 156 mhz to 320 mhz
    //
    // According to the Philips LPC2106 manual: Fcco = cclk * 2 * P    where:   Fcco = CCO frequency
    //                                                                          cclk = 53236800 hz
    //                                                                          P = PLL divisor (bits 5-6 of PLLCFG)
    //
    // Solving: Fcco = 58982400 * 2 * P
    //          P = 2  (trial value)
    //          Fcco = 58982400 * 2 * 2
    //          Fcc0 = 235929600 hz    (good choice for P since it's within the 156 mhz to 320 mhz range
    //
    // From Table 19 (page 48) of Philips LPC2106 manual    P = 2, PLLCFG bits 5-6 = 1  (assign 1 to these bits)
    //
    // Finally:      PLLCFG = 0  01  00011  =  0x23
    //
    // Final note: to load PLLCFG register, we must use the 0xAA followed 0x55 write sequence to the PLLFEED register
    //             this is done in the short function feed() below
    //

    // Setting Multiplier and Divider values
    PLLCFG=0x23;
    feed();

    // Enabling the PLL */
    PLLCON=0x1;
    feed();

    // Wait for the PLL to lock to set frequency
    while(!(PLLSTAT & PLOCK)) ;

    // Connect the PLL as the clock source
    PLLCON=0x3;
    feed();

    // Enabling MAM and setting number of clocks used for Flash memory fetch (4 cclks in this case)
    MAMCR=0x2;
    MAMTIM=0x4;

    // Setting peripheral Clock (pclk) to System Clock (cclk)
    VPBDIV=0x1;

    /* Disable the power for all pheripherials */
    PCONP &= ~((1 << 1) | (1 << 2) | (1 << 4) | (1 << 7) | \
             (1 << 8) | (1 << 9) | (1 << 10) | (1 << 12) | (1 << 19) | \
             (1 << 22) | (1 << 23));

    /* VIC initialization */
    VICINTENCLR = 0xFFFF; /* first disable all interrupts at the VIC */
    VICSOFTINTCLR = 0xFFFF; /* clear soft interrupts */
    VICINTSEL = 0; /* reset all interrupts as IRQ (not FIQ) */

    for(i=0; i<16; i++) /* reset all vectors in the VIC */
    {
        (&VICVECTCNTL0)[i] = 0;
        (&VICVECTADDR0)[i] = 0;
    }

    VICDEFVECTADDR = 0; /* set default handler */
}
#endif

void system_go_idle (void)
{
/* Idle mode - when 1, this bit causes the processor clock to be stopped,
 * while on-chip peripherals remain active. Any enabled interrupt from a
 * peripheral or an external interrupt source will cause the processor to
 * resume execution.
 */
    PCON = 1;
}
