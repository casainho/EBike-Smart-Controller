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

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "throttle.h"

void pwm_init (void)
{
  /* Time Base configuration */
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 1199; // 24MHz clock, 24MHz/1200 = 20KHz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Channel 1, 2,3 Configuration in PWM mode */
  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_Pulse = 0; // start with 0% duty cycle
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);

  /* configure ETR for current control */
  TIM_ETRConfig (TIM1, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
  //TIM_SelectInputTrigger (TIM1, TIM_TS_ETRF);
  TIM_ClearOC1Ref (TIM1, TIM_OCClear_Enable);
  TIM_ClearOC2Ref (TIM1, TIM_OCClear_Enable);
  TIM_ClearOC3Ref (TIM1, TIM_OCClear_Enable);

  /* TIM1 counter enable */
  TIM_Cmd (TIM1, ENABLE);

  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs (TIM1, ENABLE);
}

void update_duty_cycle (unsigned int value)
{
  /* value: 0 -> 1000; 0 == 0% and 1000 == 100% duty cycle
   *
   * 100% --> 1200
   * 0.1% --> 1.2
   *
   */
  value = 1200 - (value * 1.2f);
  if (value <= 0)
  {
    value = 0;
  }
  else if (value >= 1200)
  {
    value = 1200;
  }

  TIM_SetCompare1(TIM1, value);
  TIM_SetCompare2(TIM1, value);
  TIM_SetCompare3(TIM1, value);
}

void TIM1_BRK_IRQHandler (void)
{
  // reset cruise control
  cruise_control_reset ();

  // clear interrupt flag
  TIM_ClearITPendingBit (TIM1, TIM_IT_Break);
}

void brake_init (void)
{
  /* Automatic Output enable, Break, dead time and lock configuration*/
  TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
  TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Disable;
  TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Disable;
  TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
  TIM_BDTRInitStructure.TIM_DeadTime = 0;
  TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
  TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
  TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable Break interrupt */
  TIM_ITConfig (TIM1, TIM_IT_Break, ENABLE);
}
