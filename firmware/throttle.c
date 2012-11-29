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

#include "config.h"
#include "adc.h"

void throttle_init (void)
{
  adc_init (THROTTLE);
}

unsigned int throttle_get_percent (void)
{
  unsigned int value;

  // get ADC value
  value = adc_read(THROTTLE);

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
