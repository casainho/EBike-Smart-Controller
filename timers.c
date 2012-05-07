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
#include "pwm.h"
#include "ios.h"

volatile unsigned char state = 0;

void timer0_int_handler (void) __attribute__ ((interrupt("IRQ")));

void timer0_init (void)
{
  /* Initialize VIC */
  VICINTSEL &= ~(1 << 4); /* Timer 0 selected as IRQ */
  VICINTEN |= (1 << 4); /* Timer 0 interrupt enabled */
  VICVECTCNTL1 = 0x24; /* Assign Timer0; IRQ */
  VICVECTADDR1 = (unsigned long) timer0_int_handler; /* Address of the ISR */

  /* Timer/Counter 0 power/clock enable */
  PCONP |= (1 << 1);

  /* Initialize Timer 0 */
  TIMER0_TCR = 0;
  TIMER0_TC = 0; /* Counter register: Clear counter */
  TIMER0_PR = 47; /* Prescaler register: Timer0 Counter increments each 1us; 1us/(48MHz-1) */
  TIMER0_PC = 0; /* Prescaler counter register: Clear prescaler counter */

  /* Clear the interrupt flag */
  TIMER0_IR = 1;
  VICVECTADDR = 0xff;

  TIMER0_MCR = 3; /* Reset and interrupt on match */
}

void timer0_start (void)
{
  /* Start timer */
  TIMER0_TCR = 1;
}

void timer0_stop(void)
{
  /* Stop timer */
  TIMER0_TCR = 0;
}

void timer0_set_us (unsigned long us)
{
  /* Match register 0:
   * Fclk = 48000000Hz; 48MHz/48 = 1MHz -> 1us.
   * x us * 1us = x us */
  TIMER0_MR0 = us;
}

void timer0_int_handler (void)
{
  /* Clear the interrupt flag */
  TIMER0_IR = 1;
  VICVECTADDR = 0xff;

  /*             0   1                        0   1
   *           ________                      _________
   * PWM:      |||||||||  2              5  ||||||||||
   * Disable:           ____            ____
   *                        |  3   4   |
   * Enable off:             __________
   */

  switch (state)
  {
    case 0: // 0º to 60º
    //pwm_on (CHANNEL_0);
    /* Initialize Pin Select Block for MAT1.0 */
    PINSEL0 |= (1<<25);

    //phase_b_enable_off ();
    IOSET = (1 << 1);

    //pwm_off (CHANNEL_2);
    PINSEL1 &= ~(1<<7);
    IOCLR = (1<<19);

    state = 1;
    break;

    case 1: // 60º to 120º
    //phase_b_disable_off ();
    IOCLR = (1 << 1);

    //phase_c_enable_off ();
    IOSET = (1 << 2);

    state = 2;
    break;

    case 2: // 120º to 180º
    //pwm_off (CHANNEL_0);
    PINSEL0 &= ~(1<<25);
    IOCLR = (1<<12);

    //pwm_on (CHANNEL_1);
    /* Initialize Pin Select Block for MAT1.1 */
    PINSEL0 |= (1<<27);

    //phase_c_enable_off ();
    IOSET = (1 << 2);

    state = 3;
    break;

    case 3: // 180º to 240º
    //phase_a_enable_off ();
    IOSET = (1 << 0);

    //phase_c_disable_off ();
    IOCLR = (1 << 2);

    state = 4;
    break;

    case 4: // 240º to 300º
    //pwm_off (CHANNEL_1);
    PINSEL0 &= ~(1<<27);
    IOCLR = (1<<13);

    //pwm_on (CHANNEL_2);
    /* Initialize Pin Select Block for MAT1.2 */
    PINSEL1 |= (1<<7);

    state = 5;
    break;

    case 5: // 300º to 360º
    //phase_a_disable_off ();
    IOCLR = (1 << 0);

    //phase_b_enable_off ();
    IOSET = (1 << 1);

    state = 0;
    break;
  }
}
