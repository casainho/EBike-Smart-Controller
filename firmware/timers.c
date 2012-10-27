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
#include "bldc.h"

unsigned int timer0_count;

//just use it to increase the time
void __attribute__ ((interrupt("IRQ"))) timer0_int_handler (void)
{
  /* "Read" all sensors sequence and execute the BLDC coils commutation */
  commutation ();

  /* Save current timer value (time between each hall sensor signal change) */
  timer0_count = TIMER0_TC;
  TIMER0_TC = 0; // reset timer 0 value

  /* Clear the interrupt flag */
  TIMER0_IR = 1;
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
  /* Initialize VIC */
  VICINTSEL &= ~(1 << 4); /* Timer 0 selected as IRQ */
  VICVECTCNTL0 = ((1<<5) | 4); /* Assign Timer0; IRQ. Higher priority */
  VICVECTADDR0 = (unsigned long) timer0_int_handler; /* Address of the ISR */

  /* Timer/Counter 0 power/clock enable */
  PCONP |= (1 << 4);

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
  TIMER0_IR = 1;
  VICVECTADDR = 0xff;
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
