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

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

/*
 * IOs used:
 * PB13 (TIM1_CH1N)     -- PWM 4
 * PB14 (TIM1_CH2N)     -- PWM 5
 * PB15 (TIM1_CH3N)     -- PWM 6
 * PB12 (TIM1_BKIN)     -- brake signal
 * PB5  (GPIO)          -- LED debug
 * PB1  (GPIO)          -- switch for debug
 */

void gpio_init (void)
{
  /* Enable GPIOB clock. */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Turn off port bits */
  GPIO_ResetBits(GPIOB, GPIO_Pin_5 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void debug_on (void)
{
  GPIO_SetBits(GPIOB, GPIO_Pin_5);
}

void debug_off (void)
{
  GPIO_ResetBits(GPIOB, GPIO_Pin_5);
}

unsigned int switch_is_set (void)
{
  return (GPIO_ReadInputDataBit (GPIOB, GPIO_Pin_1));
}
