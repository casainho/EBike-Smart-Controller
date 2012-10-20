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

void ios_init (void)
{
  /*
    phase_a --> P0.0
    phase_b --> P0.1
    phase_c --> P0.2

    debug --> P0.4
  */

  /* P0.0, P0.1 and P0.2 as GPIOs at reset */

  /* Clear the bits for the outputs */
  IOCLR |= ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 4));

  /* Define all lines as outputs */
  IODIR |= ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 4));
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
  IOCLR = (1 << 0);
}

void phase_a_enable_off (void)
{
  IOSET = (1 << 0);
}

void phase_b_disable_off (void)
{
  IOCLR = (1 << 1);
}

void phase_b_enable_off (void)
{
  IOSET = (1 << 1);
}

void phase_c_disable_off (void)
{
  IOCLR = (1 << 2);
}

void phase_c_enable_off (void)
{
  IOSET = (1 << 2);
}

void debug_on (void)
{
  IOSET = (1 << 4);
}

void debug_off (void)
{
  IOCLR = (1 << 4);
}
