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

/* Connetions:
 *
 * PA0  (ADC1_IN1)      -- throttle signal
 * PA1  (ADC1_IN2)      -- voltage signal
 * PA2  (ADC1_IN3)      -- current signal
 * PA3  (ADC1_IN4)      -- temperature signal
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

#include <stdint.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "core_cm3.h"
#include "gpio.h"
#include "adc.h"
#include "pwm.h"
#include "bldc_hall.h"
#include "throttle.h"
#include "hall_sensor.h"

unsigned int _ms;


void delay_ms (unsigned int ms)
{
  _ms = 0;
  while (ms >= _ms) ;
}

void SysTick_Handler(void) // runs every 100ms
{
  static uint16_t cnt = 0;
  static uint8_t flip = 1;

  // for flash the LED using Throttle value
  //volatile unsigned int value = (adc_get_throttle_value () >> 4);

  // for flash LED using motor speed
  volatile unsigned int value = (get_hall_sensors_us () / 200);

  cnt++;
  if (cnt >= value && flip)
  {
    // PB5
    GPIO_SetBits(GPIOB, GPIO_Pin_5);
    cnt = 0;
    flip = !flip;
  }
  else if (cnt >= value && !flip)
  {
    // PB5
    GPIO_ResetBits(GPIOB, GPIO_Pin_5);
    cnt = 0;
    flip = !flip;
  }

  // for delay_ms ()
  _ms++;

  // read throttle value and update PWM duty cycle every 100ms
  update_duty_cycle (throttle_get_percent ()); // 1000 --> 100%
}

void initialize (void)
{
  gpio_init ();
  while (switch_is_set ()) ; // wait
  adc_init ();
  pwm_init ();
  hall_sensor_init ();

  /* Setup SysTick Timer for 1 millisecond interrupts, also enables Systick and Systick-Interrupt */
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    /* Capture error */
    while (1);
  }
}

int main (void)
{
  initialize ();

  while (1)
  {

  }

  // should never arrive here
  return 0;
}
