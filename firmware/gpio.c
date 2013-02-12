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

/* Connetions:
 *
 * PA0  (ADC1_IN1)      -- throttle signal
 * PA1  (ADC1_IN2)      -- voltage signal
 * PA2  (ADC1_IN3)      -- current signal
 * PA3  (ADC1_IN4)      -- temperature signal
 * PA4  (DAC1_OUT)      -- DAC1 signal used for current control
 * PA12 (TIM1_ETR)      -- current control input signal
 * PB12 (TIM1_BKIN)     -- brake signal
 * PA8  (TIM1_CH1)      -- PWM 1
 * PA9  (TIM1_CH2)      -- PWM 2
 * PA10 (TIM1_CH3)      -- PWM 3
 * PB13 (TIM1_CH1N)     -- PWM 4
 * PB14 (TIM1_CH2N)     -- PWM 5
 * PB15 (TIM1_CH3N)     -- PWM 6
 * PA6  (TIM3_CH1)      -- Hall sensor 1
 * PA7  (TIM3_CH2)      -- Hall sensor 2
 * PB0  (TIM3_CH3)      -- Hall sensor 3
 * PB10 (USART3_TX)     -- UART TX Bluetooth module
 * PB11 (USART3_RX)     -- UART RX Bluetooth module
 * PB5  (GPIO)          -- LED debug
 * PB1  (GPIO)          -- switch for debug
 */

void gpio_init (void)
{
  /* Enable GPIOB clock. */
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO  |
                          RCC_APB2Periph_GPIOA |
                          RCC_APB2Periph_GPIOB |
                          RCC_APB2Periph_TIM1  |
                          RCC_APB2Periph_ADC1
                          , ENABLE);

  /* DAC Periph clock enable */
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_DAC   |
                          RCC_APB1Periph_TIM3
                          , ENABLE);


  /* Once the DAC channel is enabled, the corresponding GPIO pin is automatically
     connected to the DAC converter. In order to avoid parasitic consumption,
     the GPIO pin should be configured in analog */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* GPIOA Configuration: Channel 1, 2 and 3 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Turn off port bits */
  GPIO_ResetBits(GPIOB, GPIO_Pin_5 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
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
