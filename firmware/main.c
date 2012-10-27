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
#include "system.h"
#include "isrsupport.h"
#include "timers.h"
#include "pwm.h"
#include "main.h"
#include "ios.h"
#include "bldc.h"

void initialize (void)
{
  system_init (); // initialize the LPC2103 (clocks, flash memory, etc)
  ios_init (); // configure IO pins
  pwm_init (); // initialize PWM (uses timer1)
  timer0_capture_init (); // intialize Timer0, use it for capture the Hall sensors signal time and BLDC control
  timer3_init (); // intialize timer3 (used for delay function)
  enableIRQ (); // enable interrupts
}

int main (void)
{
  initialize ();

  motor_set_duty_cycle (100);
  motor_start ();

  while (1)
  {
    debug_on ();
    delay_us (100000);

    debug_off ();
    delay_us (33000);
  }
}
