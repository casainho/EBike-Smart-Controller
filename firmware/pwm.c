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
#include "config.h"
#include "adc.h"
#include "bldc_hall.h"
#include "motor.h"

unsigned int pwm_duty_cycle = 1000;
unsigned int motor_disable = 0;

//just use it to increase the time
void __attribute__ ((interrupt("IRQ"))) pwm_int_handler (void)
{
  unsigned int adc_value = 0;

  TIMER1_MR0 = pwm_duty_cycle;
  TIMER1_MR1 = pwm_duty_cycle;
  TIMER1_MR2 = pwm_duty_cycle;

  if (motor_disable)
  {
    motor_start (); // initialize the needed interrupt
    motor_disable = 0;
  }

  // 4us delay to avoid ringing (measured on oscilloscope)
  unsigned int i;
  for (i = 0; i < 30; i++)
  {
    asm("");
  }

  debug_on ();
  // read current
  adc_value = adc_read (CURRENT);
  if (adc_value > 548) // 5 amps
  {
    phase_a_l_pwm_off ();
    phase_b_l_pwm_off ();
    phase_c_l_pwm_off ();
    motor_disable = 1;
  }
  debug_off ();

  debug_on ();
  // read current
  adc_value = adc_read (CURRENT);
  if (adc_value > 548) // 5 amps
  {
    phase_a_l_pwm_off ();
    phase_b_l_pwm_off ();
    phase_c_l_pwm_off ();
    motor_disable = 1;
  }
  debug_off ();

  /* Clear the interrupt flag */
  TIMER1_IR |= (1 << 2); // Interrupt flag for match channel 3
  VICVECTADDR = 0xff;
}

void pwm_init(void)
{
  /* Enable power for TIMER1 */
  PCONP |= (1 << 2);

  /* Initialize VIC */
  VICINTSEL &= ~(1 << 5); /* Timer 1 selected as IRQ */
  VICINTEN |= (1 << 5); /* Timer 0 interrupt enabled */
  VICVECTCNTL1 = ((1<<5) | 5); /* Assign Timer 1 IRQ */
  VICVECTADDR1 = (unsigned long) pwm_int_handler; /* Address of the ISR */

  /* CPU clock = peripheral clock = 60000000Hz */
  TIMER1_PR = 2; //timer1 clock will be 20MHz = peripheral clock / (2 + 1)

  TIMER1_MCR |= ((1<<6) | (1<<10)) ; // Interrupt on MR2: an interrupt is generated when MR2 matches the value in the TC.
   // reset timer1 on MR3 match

  TIMER1_MR3 = (1000 - 1); // PWM frequency = 20MHz / 1000 = 20kHz

  TIMER1_MR0 = 1000; // duty cycle for channel 0 = 0%
  TIMER1_MR1 = 1000; // duty cycle for channel 1 = 0%
  TIMER1_MR2 = 1000; // duty cycle for channel 2 = 0%

  TIMER1_PWMCON = (1<<0) | (1<<1) | (1<<2); // enable PWM mode for MAT1.0, MAT1.1, MAT1.2

  TIMER1_PC = 0; // reset prescale counter
  TIMER1_TC = 0; // reset timer1
  TIMER1_TCR = 1; // start counter

  /* Clear the interrupt flag */
  TIMER1_IR |= (1 << 2); // Interrupt flag for match channel 3
  VICVECTADDR = 0xff;
}

void update_duty_cycle(unsigned int value)
{
  if (pwm_duty_cycle == 1000)
  {
    TIMER1_MR0 = 1000 - value;
    TIMER1_MR1 = 1000 - value;
    TIMER1_MR2 = 1000 - value;
    pwm_duty_cycle = 1000 - value;
  }
  else
  {
    pwm_duty_cycle = 1000 - value;
  }
}
