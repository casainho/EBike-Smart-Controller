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

#include "stm32f10x_dac.h"
#include "config.h"

void motor_set_max_current (float max_current)
{
  // set the DAC output voltage to be equal to current sensor voltage for max current
  DAC_SetChannel1Data (DAC_Align_12b_R, (MOTOR_CURRENT_ZERO_AMPS_ADC_VALUE + (max_current/MOTOR_CURRENT_PER_ADC_STEP)));

  DAC_SoftwareTriggerCmd (DAC_Channel_1, ENABLE);
}
