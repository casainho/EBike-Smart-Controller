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

/*
 * PB12 (TIM1_BKIN)     -- brake signal
 * PA8  (TIM1_CH1)      -- PWM 1
 * PA9  (TIM1_CH2)      -- PWM 2
 * PA10 (TIM1_CH3)      -- PWM 3
 * PB13 (TIM1_CH1N)     -- PWM 4
 * PB14 (TIM1_CH2N)     -- PWM 5
 * PB15 (TIM1_CH3N)     -- PWM 6
 */

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

void pwm_init (void)
{
  /* TIM1, GPIOA, GPIOB clocks enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA |
                            RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  /* GPIOA Configuration: Channel 1, 2 and 3 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  uint16_t TimerPeriod = 0;
  uint16_t Channel1Pulse = 0, Channel2Pulse = 0, Channel3Pulse = 0, Channel4Pulse = 0;
  /* Compute the value to be set in ARR regiter to generate signal frequency at 17.57 Khz */
  TimerPeriod = (SystemCoreClock / 17570 ) - 1;
  /* Compute CCR1 value to generate a duty cycle at 50% for channel 1 and 1N */
  Channel1Pulse = (uint16_t) (((uint32_t) 5 * (TimerPeriod - 1)) / 10);
  /* Compute CCR2 value to generate a duty cycle at 37.5%  for channel 2 and 2N */
  Channel2Pulse = (uint16_t) (((uint32_t) 375 * (TimerPeriod - 1)) / 1000);
  /* Compute CCR3 value to generate a duty cycle at 25%  for channel 3 and 3N */
  Channel3Pulse = (uint16_t) (((uint32_t) 25 * (TimerPeriod - 1)) / 100);

  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Channel 1, 2,3 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = Channel2Pulse;
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);

  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);

  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void update_duty_cycle(unsigned int value)
{
  TIM_SetCompare1(TIM1, value);
  TIM_SetCompare2(TIM1, value);
  TIM_SetCompare3(TIM1, value);
}
