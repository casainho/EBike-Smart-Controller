/*
 * EBike Smart Controller
 *
 * Copyright (C) Jorge Pinto aka Casainho, 2012, 2013.
 *
 *   casainho [at] gmail [dot] com
 *     www.casainho.net
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_dac.h"
#include "core_cm3.h"
#include "gpio.h"
#include "adc.h"
#include "pwm.h"
#include "bldc.h"
#include "throttle.h"
#include "hall_sensor.h"
#include "dac.h"
#include "config.h"
#include "motor.h"

volatile unsigned int _ms;

void delay_ms (unsigned int ms)
{
  _ms = 1;
  while (ms >= _ms) ;
}

void SysTick_Handler(void) // runs every 1ms
{
  // need to call this every 1ms
  cruise_control_tick ();

  // for delay_ms ()
  _ms++;

  // read throttle value and update PWM duty cycle every 100ms
  update_duty_cycle (throttle_get_percent ()); // 1000 --> 100%
}

void initialize (void)
{
  gpio_init ();
  while (switch_is_set ()) ; // wait
  adc_init ();
  dac_init ();
  brake_init ();
  commutation_disable ();
  pwm_init ();
  hall_sensor_init ();

  /* Setup SysTick Timer for 1 millisecond interrupts, also enables Systick and Systick-Interrupt */
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    /* Capture error */
    while (1);
  }
}

int main (void)
{
  initialize ();

  motor_set_max_current (0.3f); // set max current in amps

  volatile unsigned int time = 0, a = 0;

  while (1)
  {
    time = get_hall_sensors_us ();
    if (time > a)
      {
        a = time;
      }
  }

  // should never arrive here
  return 0;
}
