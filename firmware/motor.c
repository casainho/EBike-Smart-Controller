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
#include "bldc_hall.h"
#include "pwm.h"
#include "timers.h"
#include "adc.h"

float _current_max = 0;

unsigned int motor_get_speed (void)
{
  unsigned int motor_speed;
  motor_speed = 1.0 / ((get_timer0_count () * 6.0) / 1000000.0);
  return motor_speed;
}

void motor_set_speed (unsigned int speed)
{
  (void) speed;
}

void motor_start (void)
{
  VICINTEN |= (1 << 4); /* Timer 0 interrupt enabled */
  commutate (); // starts the commutation
}

void motor_coast (void)
{
  VICINTEN &= ~(1 << 4); /* Timer 0 interrupt disabled, stops the commutation */
  commutation_disable ();  // disable PWM
}

void motor_set_duty_cycle (unsigned int value)
{
  update_duty_cycle (value);
}

float motor_get_current (void)
{
  unsigned int adc_value;
  float current_value;

  adc_value = adc_read (CURRENT);
  if (adc_value < 507)
  {
    adc_value = 0; // put to zero negative values of current readed by ACS756
  }
  else
  {
    adc_value -= 508; // shift value to zero
  }

  current_value = (float) (adc_value * MOTOR_CURRENT_PER_ADC_STEP);
  return current_value;
}

void motor_current_control (unsigned int duty_cycle)
{
  static unsigned int _duty_cycle = 0;
  float current;

  current = motor_get_current ();
  if (current > _current_max)
  {
    if (_duty_cycle > 10)
    {
      _duty_cycle -= 10;
    }
    else if (_duty_cycle > 5)
    {
      _duty_cycle -= 5;
    }
    else if (_duty_cycle > 1)
    {
      _duty_cycle -= 1;
    }
  }
  else // (current < _current_max)
  {
    if (_duty_cycle < duty_cycle)
    {
      _duty_cycle++;
    }
  }

  motor_set_duty_cycle (_duty_cycle);

  return;
}

void motor_set_current_max (float current_max)
{
  _current_max = current_max;
}
