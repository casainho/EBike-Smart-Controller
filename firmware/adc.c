/*
 * EBike Smart Controller
 *
 * Copyright (C) Jorge Pinto aka Casainho, 2012.
 *
 *   casainho [at] gmail [dot] com
 *     www.casainho.net
 *
 * Released under the GPL License, Version 3
 */

#include "lpc210x.h"

void adc_init (void)
{
    /* Enable the ADC pheripherial power */
    PCONP |= (1 << 12);

    /* Select P0.22 to be used for ADC */
    PINSEL1 |=  ((1 << 12) | (1 << 13)) \ // AD0.0
                ((1 << 14) | (1 << 15)) \ // AD0.1
                ((1 << 16) | (1 << 17));  // AD0.2
}

unsigned short int adc_read (unsigned char channel)
{
    /* Enable ADC; configure the clock; 10 bits resolution; configure channel */
    /* CLKDIV = 12 ==> > 4.5MHz the clock for ADC */
    /* Each conversion should take about 2,5us */
    ADCR = ((12 << 8) | (1 << 21) | (1 << 24) | (1 << channel));

    /* Wait for finish the conversion */
    while (!(ADGDR & (1 << 31))) ;

    /* Return the value (10 bits) */
    return ((ADGDR >> 6) & 0x3ff);
}
