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
#include "ios.h"

#define PHASE_A 7
#define PHASE_B 8
#define PHASE_C 9

#define DEBUG 9
#define SWITCH 3

void ios_init (void)
{
  /*
    phase_a --> P0.7
    phase_b --> P0.9
    phase_c --> P0.8

    debug --> P0.4
  */

  /* P0.7, P0.8 and P0.9 as GPIOs at reset */

  /* Clear the bits for the outputs */
  IOCLR |= ((1 << PHASE_A) | (1 << PHASE_B) | (1 << PHASE_C) /*| (1 << DEBUG) |*/);

  /* Define all lines as outputs */
  IODIR |= ((1 << PHASE_A) | (1 << PHASE_B) | (1 << PHASE_C) /*| (1 << DEBUG) |*/);
}

unsigned char io_is_set (unsigned char io_number)
{
  if ((IOPIN >> io_number) & 1)
    return 1;

  else
    return 0;
}

unsigned long int get_ios (void)
{
  return IOPIN;
}

void debug_on (void)
{
  IOSET = (1 << DEBUG);
}

void debug_off (void)
{
  IOCLR = (1 << DEBUG);
}

unsigned char switch_is_set (void)
{
  if (io_is_set (SWITCH))
    return 1;

  else
    return 0;
}
