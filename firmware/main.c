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
  adc_init (CURRENT); // init the ADC for current measure
  adc_init (VOLTAGE); // init the ADC for current measure
  pwm_init (); // initialize PWM (uses timer1)
  //timer0_capture_init (); // intialize Timer0, use it for capture the BEMF sensors signal time and BLDC control
  //timer2_init ();
  timer3_init (); // intialize timer3 (used for delay function)
  enableIRQ (); // enable interrupts
}

unsigned int sector_increment (unsigned int sector)
{
  if (sector < 6)
    sector++;
  else
    sector = 1;

  return sector;
}

unsigned int sector_decrement (unsigned int sector)
{
  if (sector > 1)
    sector--;
  else
    sector = 6;

  return sector;
}

int main (void)
{
  unsigned int duty_cycle;

  initialize ();

  //initial_duty_cycle = duty_cycle = 300;
  //motor_set_duty_cycle (initial_duty_cycle);
  //motor_start ();

  static unsigned int sector, i;

  duty_cycle = 500;
  motor_set_duty_cycle (duty_cycle);
  while (1)
  {

    for (i = 0; i < 6; i++)
    {
      // find sector
      sector = rotor_find_position_sector ();

      // increment, step+2
      sector = sector_increment (sector);
      sector = sector_increment (sector);

      // step
      motor_set_duty_cycle (250);
      commutation_sector (sector);

      // wait 5ms
      delay_us (20000);
      commutation_disable ();
      delay_us (20000);
    }

    // wait 1s
    delay_us (1000000);











#if 0
    // wait 5ms
    for (i = 0; i < 400; i++)
    {
      delay_us (10);

      // verify max current
      current = adc_read (CURRENT);
      if (current > 650)
      {
        duty_cycle = 0;
        end = 1;
#if 0
        if (duty_cycle > 25)
        {
          duty_cycle -= 25;
        }
        else if (duty_cycle > 0)
        {
          duty_cycle -= 1;
        }
#endif
      }
      else
      {
        if ((duty_cycle < 500) && (end = 0))
        {
          duty_cycle++;
        }
      }
      motor_set_duty_cycle (duty_cycle);
    }
    commutation_disable ();

    delay_us (1000);

#endif




























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
