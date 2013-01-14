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
#include "config.h"
#include "ios.h"
#include "adc.h"
#include "bldc_hall.h"
#include "motor.h"
#include "throttle.h"

extern unsigned int motor_status;

void initialize (void)
{
  system_init (); // initialize the LPC2103 (clocks, flash memory, etc)
  ios_init (); // configure IO pins
  while (switch_is_set ()) ; // wait
  adc_init (CURRENT); // init the ADC for current measure
  throttle_init (); // init the ADC for thottle
  pwm_init (); // initialize PWM (uses timer1) (PWM output pins will be disable)
  timer0_capture_init (); // intialize Timer0, use it for capture the BEMF sensors signal time and BLDC control
  timer3_init (); // intialize timer3 (used for delay function)
  enableIRQ (); // enable interrupts
}

int main (void)
{
  volatile unsigned int duty_cycle = 0;

  initialize ();

  while (1)
  {
    duty_cycle = throttle_get_percent (); // get throttle value
    if (duty_cycle < 250)
    {
      duty_cycle = 0;
    }

    // if throttle is off, coast
    if (duty_cycle == 0)
    {
      motor_coast ();
    }
    // if throttle is active with some value and motor was in coast state, start the motor!!
    else if (duty_cycle != 0 && motor_is_coast())
    {
      motor_set_duty_cycle (duty_cycle);
      motor_start ();
    }
    // if throttle is active with some value and motor is running, keep it running and update duty_cycle = throttle value
    else
    {
      motor_set_duty_cycle (duty_cycle);
    }
  }
}
