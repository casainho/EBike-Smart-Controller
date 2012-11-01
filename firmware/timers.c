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
#include "isrsupport.h"
#include "main.h"
#include "pwm.h"
#include "ios.h"
#include "bldc.h"

unsigned int timer0_count;

//just use it to increase the time
void __attribute__ ((interrupt("IRQ"))) timer0_int_handler (void)
{
  /* "Read" all sensors sequence and execute the BLDC coils commutation */
  //commutation ();

  /* Save current timer value (time between each hall sensor signal change) */
  timer0_count = TIMER0_TC;
  TIMER0_TC = 0; // reset timer 0 value

  /* Clear the interrupt flag */
  TIMER0_IR |= ((1 << 4) | (1 << 5) | (1 << 6));
  VICVECTADDR = 0xff;
}

unsigned int get_timer0_count (void)
{
  unsigned int temp;

  disableIRQ (); // disable interrupts
  temp = timer0_count; // atomic access
  enableIRQ (); // enable interrupts
  return temp;
}

void timer0_capture_init (void)
{
  // P0.2, P0.4, P0.6 as Capture input pins
  PINSEL0 |= ((1 << 5) | (1 << 9) | (1 << 13));

  /* Initialize VIC */
  VICINTSEL &= ~(1 << 4); /* Timer 0 selected as IRQ */
  VICVECTCNTL0 = ((1<<5) | 4); /* Assign Timer0; IRQ. Higher priority */
  VICVECTADDR0 = (unsigned long) timer0_int_handler; /* Address of the ISR */

  /* Timer/Counter 0 power/clock enable */
  PCONP |= (1 << 1);

  /* Initialize Timer 0 */
  TIMER0_TCR = 0;
  TIMER0_TC = 0; /* Counter register: Clear counter */
  TIMER0_PR = 47; /* Prescaler register: Timer0 Counter increments each 1us; 1us/(48MHz-1) */
  TIMER0_PC = 0; /* Prescaler counter register: Clear prescaler counter */

  /* Start timer */
  TIMER0_TCR = 1;

  // Interrupt is generated on each hall sensors signal change (CAP0.0; CAP0.1and CAP0.2)
  TIMER0_CCR = 0x1FF;

  /* Clear the interrupt flag */
  TIMER0_IR |= ((1 << 4) | (1 << 5) | (1 << 6));
  VICVECTADDR = 0xff;
}

//just use it to increase the time
void __attribute__ ((interrupt("IRQ"))) timer2_int_handler (void)
{
  /* Execute the BLDC coils commutation */
  //commutation ();

  /* Clear the interrupt flag */
  TIMER2_IR = 1;
  VICVECTADDR = 0xff;
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

  TIMER2_MCR = 3; /* Reset and interrupt on match */
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


void timer3_init (void)
{
  /* Timer/Counter 3 power/clock enable */
  PCONP |= (1 << 23);

  /* Initialize Timer 3 */
  TIMER3_TCR = 0;
  TIMER3_TC = 0; /* Counter register: Clear counter */
  TIMER3_PR = 47; /* Prescaler register: Timer3 Counter increments each 1us; 1us/(48MHz-1) */
  TIMER3_PC = 0; /* Prescaler counter register: Clear prescaler counter */

  /* Start timer */
  TIMER3_TCR = 1;
}

/* Atomic */
unsigned long micros(void)
{
  return TIMER3_TC;
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
