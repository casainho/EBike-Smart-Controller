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
#include "hall_sensor.h"
#include "config.h"
#include "bldc.h"
#include "pwm.h"

void motor_set_max_current (float max_current)
{
  // set the DAC output voltage to be equal to current sensor voltage for max current
  DAC_SetChannel1Data (DAC_Align_12b_R, (MOTOR_CURRENT_ZERO_AMPS_ADC_VALUE + (max_current/MOTOR_CURRENT_PER_ADC_STEP)));

  DAC_SoftwareTriggerCmd (DAC_Channel_1, ENABLE);
}

unsigned int motor_get_speed (void)
{
  unsigned int motor_speed;
  motor_speed = 1.0 / ((get_hall_sensors_us () * 6.0) / 1000000.0);
  return motor_speed;
}

void motor_set_speed (unsigned int speed)
{
  (void) speed;
}

void motor_start (void)
{
  commutate (); // starts the commutation

  /* PWM Output Enable */
  TIM_CtrlPWMOutputs (TIM1, ENABLE);
}

void motor_coast (void)
{
  /* PWM Output Disable */
  TIM_CtrlPWMOutputs (TIM1, DISABLE);
}

void motor_set_duty_cycle (unsigned int value)
{
  update_duty_cycle (value);
}
