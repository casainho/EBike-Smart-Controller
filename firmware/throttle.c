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

#include "config.h"
#include "adc.h"
#include "stm32f10x_gpio.h"

unsigned int cruise_control = 0;
unsigned int count_cruise_control = 1;
unsigned int throttle_cruise_percent = 0;
unsigned int state = 0;

unsigned int throttle_percent_value (void)
{
  unsigned int value;

  value = adc_get_throttle_value ();

  // limit maximum
  if (value > THROTTLE_ADC_MAX)
  {
    value = THROTTLE_ADC_MAX;
    return 1000;
  }

  // limit minimum
  if (value < THROTTLE_ADC_MIN)
  {
    value = THROTTLE_ADC_MIN;
    return 0;
  }

  // shift minimum value to 0
  value = value - THROTTLE_ADC_MIN;
  // calc value in percent * 10 [0 - 1000]
  value = (value * 1000) / THROTTLE_ADC_AMPLITUDE;

  return value;
}

// Returns the value of Throttle percent (0 up to 1000, where 1000 is equal to 100%).
// Cruise Control superimposes the Throttle value.
unsigned int throttle_get_percent (void)
{
  if (cruise_control)
  {
    return throttle_cruise_percent;
  }

  return throttle_percent_value ();
}

void cruise_control_tick (void)
{
#define PERCENT_DELTA 100 // 10%

  static unsigned int throttle_last_percent = 0;
  unsigned int throttle_percent;

  throttle_percent = throttle_percent_value() + 1500; // shift 1500 so next calcs can accommodate negative values

  switch (state)
  {
    // wait for stable throttle over 5 seconds
    //
    case 0:
    if ((1500 + PERCENT_DELTA) < throttle_percent) // minimum value
    {
      if (((throttle_last_percent + PERCENT_DELTA) >= throttle_percent)
          || ((throttle_last_percent - PERCENT_DELTA) < throttle_percent))
      {
        // throttle didn't changed
        count_cruise_control++;
        GPIO_ResetBits(GPIOB, GPIO_Pin_5);; // disable LED

        if (count_cruise_control > 500) // 5 seconds
        {
          GPIO_SetBits(GPIOB, GPIO_Pin_5); // enable LED
          count_cruise_control = 1;
          cruise_control = 1;
          throttle_cruise_percent = throttle_percent - 1500;
          state = 1;
        }
      }
      else
      {
        count_cruise_control = 1;
        GPIO_ResetBits(GPIOB, GPIO_Pin_5);; // disable LED
      }
    }
    else
    {
      count_cruise_control = 1;
      GPIO_ResetBits(GPIOB, GPIO_Pin_5);; // disable LED
    }

    throttle_last_percent = throttle_percent;
    break;

    // verify if throttle value don't go up
    //
    case 1:
    if (throttle_percent > (throttle_last_percent + PERCENT_DELTA))
    {
      cruise_control = 0;
      count_cruise_control = 1;
      state = 0;
      GPIO_ResetBits(GPIOB, GPIO_Pin_5);; // disable LED
    }
    else if (throttle_percent == 1500)
    {
      state = 2;
    }
    break;

    // verify if throttle value go up
    //
    case 2:
    if (throttle_percent > (1500 + PERCENT_DELTA))
    {
      cruise_control = 0;
      count_cruise_control = 1;
      state = 0;
      GPIO_ResetBits(GPIOB, GPIO_Pin_5);; // disable LED
    }
    break;

    default:
    break;
  }
}

void cruise_control_reset (void)
{
  cruise_control = 0;
  count_cruise_control = 1;
  state = 0;
  GPIO_ResetBits(GPIOB, GPIO_Pin_5);; // disable LED
}

