/*
 * EBike Smart Controller
 *
 * Copyright (C) Jorge Pinto aka Casainho, 2012.
 *
 *   casainho [at] gmail [dot] com
 *     www.casainho.net
 *
 * Released under the GPL Licence, Version 3
 */

#include "lpc210x.h"
#include "main.h"

void timer1_int_handler (void)   __attribute__ ((interrupt("IRQ")));

long millis_ticks = 0;

void timer1_init (void)
{
    /* Initialize VIC */
    VICINTSEL &= ~(1 << 5); /* Timer 1 selected as IRQ */
    VICINTEN |= (1 << 5); /* Timer 1 interrupt enabled */
    VICVECTCNTL1 = 0x25;
    VICVECTADDR1 = (unsigned long) timer1_int_handler; /* Address of the ISR */

    /* Timer/Counter 1 power/clock enable */
    PCONP |= (1 << 2);

    /* Initialize Timer 1 */
    TIMER1_TCR = 0;
    TIMER1_TC = 0; /* Counter register: Clear counter */
    TIMER1_PR = 47; /* Prescaler register: Timer2 Counter increments each 1us; 1us/(48MHz-1) */
    TIMER1_PC = 0; /* Prescaler counter register: Clear prescaler counter */

#ifdef XTAL_12000000HZ
    /* Match register 0:
     * Fclk = 48000000Hz; 48MHz/48 = 1MHz -> 1us.
     * 1000 * 1us = 1ms */
    TIMER1_MR0 = 1000;
#elif defined XTAL_14745600HZ
#error TODO
    /* Match register 0:
     * Fclk = 53236800Hz; 1ms => 0,001/(1/53236800); 1ms => 53237 */
    TIMER1_MR0 = 53237;
#else
#ERROR XTAL frequ need to be defined
#endif

    TIMER1_MCR = 3; /* Reset and interrupt on match */

    /* Start timer */
    TIMER1_TCR = 1;
}

void timer1_int_handler (void)
{
    /* Clear the interrupt flag */
    TIMER1_IR = 1;
    VICVECTADDR = 0xff;

    millis_ticks++;
}

/* Atomic */
unsigned long millis(void)
{
  unsigned long t;
  VICINTEN &= ~(1 << 5); /* Timer 1 interrupt disabled */
  t = millis_ticks;
  VICINTEN |= (1 << 5); /* Timer 1 interrupt enabled */
  return t;
}

void delay_ms(unsigned long ms)
{
  unsigned long start = millis();
  while (millis() - start < ms)
    ;
}

/* Atomic */
long micros(void)
{
  return TIMER1_TC;
}

/* Always with ~2ms offset. delay_us(1) will be a delay of 3us */
void delay_us(unsigned long us)
{
  unsigned long start = micros();

  while (micros() - start < us)
    ;
}
