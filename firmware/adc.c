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
#include "config.h"

void adc_init (unsigned char channel)
{
  /* Enable the ADC pheripherial power */
  PCONP |= (1 << 12);

  /* Enable ADC; configure the clock; 10 bits resolution; configure channel */
  /* Clock = 60MHZ; CLKDIV = 16 => 4MHz the clock for ADC */
  /* Each conversion should take about 3us */
  /* Enable BURST mode */
  ADCR |= ((15 << 8) // clock
        | (1 << 16) // 4MHz
        | (1 << 21) // burst mode
        | (1 << channel)); // channel select

  switch (channel)
  {
    case VOLTAGE:
    PINSEL1 |= ((1 << 12) | (1 << 13)); // AD0.0
    break;

    case CURRENT:
    PINSEL1 |= ((1 << 14) | (1 << 15)); // AD0.1
    break;

    case THROTTLE:
    PINSEL1 |= ((1 << 16) | (1 << 17)); // AD0.2
    break;

    default:
    break;
  }
}

unsigned int adc_read (unsigned char channel)
{

  volatile unsigned int adc;

  adc = *((volatile unsigned long *) (0xE0034010 + (channel*4)));

  /* Return the value (10 bits) */
  return ((adc >> 6) & 0x3ff);
}
