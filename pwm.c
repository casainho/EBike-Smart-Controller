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
#include "pwm.h"
#include "ios.h"

void pwm_init(void)
{
  /*
    MAT1.0: P0.12
    MAT1.1: P0.13
    MAT1.2: P0.19
  */

  /* Enable power for TIMER1 */
  PCONP |= (1 << 2);

  /* CPU clock = peripheral clock = 48000000Hz */
  TIMER1_PR = 2; //timer0 clock will be 16MHz = peripheral clock / (2 + 1)

  TIMER1_MCR |= (1<<10); // reset timer1 on MR3 match
  TIMER1_MR3 = (1000 - 1); // PWM frequency = 16MHz / 1000 = 16kHz

  TIMER1_MR0 = 1000; // duty cycle for channel 0 = 0%
  TIMER1_MR1 = 1000; // duty cycle for channel 1 = 0%
  TIMER1_MR2 = 1000; // duty cycle for channel 2 = 0%

  TIMER1_PWMCON = (1<<0) | (1<<1) | (1<<2); // enable PWM mode for MAT1.0, MAT1.1, MAT1.2

  TIMER1_PC = 0; // reset prescale counter
  TIMER1_TC = 0; // reset timer1
  TIMER1_TCR = 1; // start counter
}

void update_duty_cycle(unsigned int value)
{
  value = 1000 - value;

  // Setup the value to the correspondent channel
  TIMER1_MR0 = value;
  TIMER1_MR1 = value;
  TIMER1_MR2 = value;
}

void pwm_on (unsigned char pwm_phase)
{
  // Setup the value to the correspondent channel
  if (pwm_phase == CHANNEL_0)
  {
    /* Initialize Pin Select Block for MAT1.0 */
    PINSEL0 |= (1<<25);
  }
  // Setup the value to the correspondent channel
  else if (pwm_phase == CHANNEL_1)
  {
    /* Initialize Pin Select Block for MAT1.1 */
    PINSEL0 |= (1<<27);
  }
  // Setup the value to the correspondent channel
  else if (pwm_phase == CHANNEL_2)
  {
    /* Initialize Pin Select Block for MAT1.2 */
    PINSEL1 |= (1<<7);
  }
}

void pwm_off (unsigned char pwm_phase)
{
  // Setup the value to the correspondent channel
  if (pwm_phase == CHANNEL_0)
  {
    PINSEL0 &= ~(1<<25);
    IOCLR = (1<<12);
  }
  // Setup the value to the correspondent channel
  else if (pwm_phase == CHANNEL_1)
  {
    PINSEL0 &= ~(1<<27);
    IOCLR = (1<<13);
  }
  // Setup the value to the correspondent channel
  else if (pwm_phase == CHANNEL_2)
  {
    PINSEL1 &= ~(1<<7);
    IOCLR = (1<<19);
  }
}
