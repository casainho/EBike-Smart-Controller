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

unsigned int throttle_get_percent (void)
{
  unsigned int value;

  // get ADC value
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
