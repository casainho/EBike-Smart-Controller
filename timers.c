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

#include "lpc210x.h"
#include "main.h"
#include "pwm.h"
#include "ios.h"
#include "motor.h"

//just use it to increase the time
void __attribute__ ((interrupt("IRQ"))) timer2_int_handler (void)
{
  /* Clear the interrupt flag */
  TIMER2_IR = 1;
  VICVECTADDR = 0xff;

  /* Execute the BLDC coils commutation */
  commutation ();
}

void timer2_init (void)
{
  /* Initialize VIC */
  VICINTSEL &= ~(1 << 26); /* Timer 2 selected as IRQ */
  VICINTEN |= (1 << 26); /* Timer 2 interrupt enabled */
  VICVECTCNTL0 = ((1<<5) | 26); /* Assign Timer2; IRQ. Higher priority */
  VICVECTADDR0 = (unsigned long) timer2_int_handler; /* Address of the ISR */

  /* Timer/Counter 2 power/clock enable */
  PCONP |= (1 << 22);

  /* Initialize Timer 2 */
  TIMER2_TCR = 0;
  TIMER2_TC = 0; /* Counter register: Clear counter */
  TIMER2_PR = 47; /* Prescaler register: Timer2 Counter increments each 1us; 1us/(48MHz-1) */
  TIMER2_PC = 0; /* Prescaler counter register: Clear prescaler counter */

  /* Clear the interrupt flag */
  TIMER2_IR = 1;
  VICVECTADDR = 0xff;

  TIMER0_MCR = 3; /* Reset and interrupt on match */
}

void timer2_start (void)
{
  /* Start timer */
  TIMER2_TCR = 1;
}

void timer2_stop(void)
{
  /* Stop timer */
  TIMER2_TCR = 0;
}

void timer2_set_us (unsigned long us)
{
  /* Match register 0:
   * Fclk = 48000000Hz; 48MHz/48 = 1MHz -> 1us.
   * x us * 1us = x us */
  TIMER2_MR0 = us;
}

void timer2_init (void)
{
  /* Timer/Counter 2 power/clock enable */
  PCONP |= (1 << 22);

  /* Initialize Timer 2 */
  TIMER2_TCR = 0;
  TIMER2_TC = 0; /* Counter register: Clear counter */
  TIMER2_PR = 47; /* Prescaler register: Timer2 Counter increments each 1us; 1us/(48MHz-1) */
  TIMER2_PC = 0; /* Prescaler counter register: Clear prescaler counter */

  /* Start timer */
  TIMER2_TCR = 1;
}

/* Atomic */
unsigned long micros(void)
{
  return TIMER2_TC;
}

void delay_us(unsigned long us)
{
  // There is some bug. Seems the delay is not consistent.

  unsigned long a = us / 65535;
  unsigned long b = us - (a * 65535);

  while (a > 0)
  {
    a--;

    unsigned long start = micros();

    while (micros() - start < 65535)
      ;
  }

  if (b > 0)
  {
    unsigned long start = micros();

    while (micros() - start < b)
      ;
  }
}
