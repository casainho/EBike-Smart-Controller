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
#include "bldc_hall.h"

unsigned int timer0_count;
unsigned int motor_status;

//just use it to increase the time
void __attribute__ ((interrupt("IRQ"))) timer0_int_handler (void)
{
  static unsigned int c = 0;

  // detect motor movement
  if (motor_status == 0)
  {
    c++;
    if (c > 2500)
    {
      motor_status = 1; // motor is running
      c = 0;
    }
  }
  else if (motor_status == 1)
  {
    /* "Read" all sensors sequence and execute the BLDC coils commutation */
    commutate ();
  }

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

void timer3_init (void)
{
  /* Timer/Counter 3 power/clock enable */
  PCONP |= (1 << 23);

  /* Initialize Timer 3 */
  TIMER3_TCR = 0;
  TIMER3_TC = 0; /* Counter register: Clear counter */
  TIMER3_PR = 479; /* Prescaler register: Timer3 Counter increments each 10us; 1us/((48*10)-1) */
  TIMER3_PC = 0; /* Prescaler counter register: Clear prescaler counter */

  /* Start timer */
  TIMER3_TCR = 1;
}

/* Atomic */
void micros10_clear(void)
{
  TIMER3_TC = 0;
}

unsigned int micros10(void)
{
  return TIMER3_TC;
}

void delay_us10(unsigned long us10)
{
  unsigned int a = us10 / 65536;
  static unsigned int b;
  b = us10 - (a * 65536);
  unsigned int c;

  for ( ; a > 0; a--)
  {
    // 2 X 32768 loops of us = 65536us
    c = micros10() + 32768;
    if (c > 65535)
    {
      c -= 65535;
    }
    while (micros10() != c) ;

    c = micros10() + 32768;
    if (c > 65535)
    {
      c -= 65535;
    }
    while (micros10() != c) ;
  }

  c = micros10() + b;
  if (c > 65535)
  {
    c -= 65535;
  }
  while (micros10() != c) ;
}
