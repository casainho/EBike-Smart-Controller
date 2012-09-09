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
#include "pwm.h"
#include "ios.h"

extern BYTE bSector;
extern BOOL fDir;

void pwm_init(void)
{
  /*
    MAT1.0: P0.12
    MAT1.1: P0.13
    MAT1.2: P0.19
  */

  // Configure P0.12, P0.13 and P0.19 for working as  MAT1.0, MAT1.1, MAT1.2
  PINSEL0 |= ((1 << 25) | (1 << 27));
  PINSEL1 |= (1 << 7);

  /* Enable power for TIMER1 */
  PCONP |= (1 << 2);

  /* CPU clock = peripheral clock = 48000000Hz */
  TIMER1_PR = 2; //timer0 clock will be 16MHz = peripheral clock / (2 + 1)

  TIMER1_MCR |= (1<<10); // reset timer1 on MR3 match
  TIMER1_MR3 = (1000 - 1); // PWM frequency = 16MHz / 1000 = 16kHz

  TIMER1_MR0 = 0; // duty cycle for channel 0 = 0%
  TIMER1_MR1 = 0; // duty cycle for channel 1 = 0%
  TIMER1_MR2 = 0; // duty cycle for channel 2 = 0%

  TIMER1_PWMCON = (1<<0) | (1<<1) | (1<<2); // enable PWM mode for MAT1.0, MAT1.1, MAT1.2

  TIMER1_PC = 0; // reset prescale counter
  TIMER1_TC = 0; // reset timer1
  TIMER1_TCR = 1; // start counter
}

void update_duty_cycle(unsigned int value)
{
  /* Duty-cycle state is inverted on hardware,
   * the output of PWM is also inverted.
   */

  // Setup the value to the correspondent channel
  TIMER1_MR0 = value;
  TIMER1_MR1 = value;
  TIMER1_MR2 = value;
}
