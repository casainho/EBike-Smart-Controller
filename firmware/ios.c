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

void ios_init (void)
{
  /*
    phase_a --> P0.7
    phase_b --> P0.8
    phase_c --> P0.9

    debug --> P0.4
  */

  /* P0.7, P0.8 and P0.9 as GPIOs at reset */

  /* Clear the bits for the outputs */
  IOCLR |= ((1 << PHASE_A) | (1 << PHASE_B) | (1 << PHASE_C));

  /* Define all lines as outputs */
  IODIR |= ((1 << PHASE_A) | (1 << PHASE_B) | (1 << PHASE_C));
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

void phase_a_disable_off (void)
{
  IOCLR = (1 << PHASE_A);
}

void phase_a_enable_off (void)
{
  IOSET = (1 << PHASE_A);
}

void phase_b_disable_off (void)
{
  IOCLR = (1 << PHASE_B);
}

void phase_b_enable_off (void)
{
  IOSET = (1 << PHASE_B);
}

void phase_c_disable_off (void)
{
  IOCLR = (1 << PHASE_C);
}

void phase_c_enable_off (void)
{
  IOSET = (1 << PHASE_C);
}

void debug_on (void)
{
  IOSET = (1 << DEBUG);
}

void debug_off (void)
{
  IOCLR = (1 << DEBUG);
}
