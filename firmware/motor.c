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

#include "bldc.h"
#include "pwm.h"

unsigned int motor_get_speed (void)
{
  unsigned int motor_speed;
  motor_speed = 1.0 / ((get_timer0_count () * 6.0) / 1000000.0);
}

unsigned int motor_set_speed (void)
{

}

void motor_start (void)
{
  VICINTEN |= (1 << 4); /* Timer 0 interrupt enabled */
  commutation (); // starts the commutation
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
