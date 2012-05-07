/*
 * EBike Smart Controller
 *
 * Copyright (C) Jorge Pinto aka Casainho, 2012.
 *
 *   casainho [at] gmail [dot] com
 *     www.casainho.net
 *
 * Released under the GPL Licence, Version 3
 */

#include "lpc210x.h"
#include "system.h"
#include "isrsupport.h"
#include "timers.h"
#include "pwm.h"
#include "main.h"
#include "ios.h"

int main (void)
{
  /* Initialize the system */
  system_init();
  timer0_init();
  enableIRQ();
  pwm_init();
  ios_init();

  // Set period 6 * 5ms = 30ms
  timer0_set_us (5000);
  // Start timer
  timer0_start ();

  // Set duty-cycle
  update_duty_cycle(2); // 0 up to 1000

  while (1)
  {

  }
}
