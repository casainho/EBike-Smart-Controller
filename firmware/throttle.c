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
  return ((unsigned int) (adc_read (THROTTLE) * THROTTLE_PERCENT_PER_ADC_STEP));
}
