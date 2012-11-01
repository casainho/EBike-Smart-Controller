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
#include "bldc.h"
#include "motor.h"

void initialize (void)
{
  system_init (); // initialize the LPC2103 (clocks, flash memory, etc)
  ios_init (); // configure IO pins
  while (switch_is_set ()) ; // wait
  //adc_init (CURRENT); // init the ADC for current measure
  pwm_init (); // initialize PWM (uses timer1)
  //timer0_capture_init (); // intialize Timer0, use it for capture the BEMF sensors signal time and BLDC control
  //timer2_init ();
  timer3_init (); // intialize timer3 (used for delay function)
  enableIRQ (); // enable interrupts
}

int main (void)
{
  unsigned int initial_duty_cycle;
  unsigned int duty_cycle;
  volatile float motor_current;

  initialize ();

  //initial_duty_cycle = duty_cycle = 300;
  //motor_set_duty_cycle (initial_duty_cycle);
  //motor_start ();

  motor_set_duty_cycle (1000);
  static int sector = 0, i;
  while (1)
  {
    sector = rotor_find_position_sector ();

    for (i = 0; i < 500; i++)
    {
      delay_us (100);
    }

#if 0
      commutation_sector_2 ();
      delay_us (200);
      commutation_disable ();
      for (i = 0; i < 1000; i++)
      {
        delay_us (1000);
      }

      commutation_sector_3 ();
      delay_us (200);
      commutation_disable ();
      for (i = 0; i < 1000; i++)
      {
        delay_us (1000);
      }

      commutation_sector_4 ();
      delay_us (200);
      commutation_disable ();
      for (i = 0; i < 1000; i++)
      {
        delay_us (1000);
      }

      commutation_sector_5 ();
      delay_us (200);
      commutation_disable ();
      for (i = 0; i < 1000; i++)
      {
        delay_us (1000);
      }

      commutation_sector_6 ();
      delay_us (200);
      commutation_disable ();
      for (i = 0; i < 1000; i++)
      {
        delay_us (1000);
      }

#endif


    while (switch_is_set ())
    {
      motor_set_duty_cycle (0);
    }
  }

  while (1)
  {
#if 0
    motor_current = motor_get_current ();
    if (motor_current > MOTOR_MAX_CURRENT)
    {
      if (duty_cycle > 0)
      {
        duty_cycle -= 1;
        motor_set_duty_cycle (duty_cycle);
      }
    }
    else if ((motor_get_current () < MOTOR_MAX_CURRENT) & (duty_cycle < initial_duty_cycle))
    {
      if (duty_cycle < 1000)
      {
        duty_cycle += 1;
        motor_set_duty_cycle (duty_cycle);
      }
    }
#endif

    while (switch_is_set ())
    {
      motor_set_duty_cycle (0);
    }
  }
}
