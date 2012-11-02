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
#include "config.h"
#include "pwm.h"
#include "ios.h"
#include "timers.h"
#include "motor.h"
#include "adc.h"

#define PHASE_A 7
#define PHASE_B 8
#define PHASE_C 9

//functions to control each of 6 PWM signals
void phase_a_h_on (void)
{
  /* LPC2103 P0.7 --> CPU1 */
  /* set to output */
  IODIR |= (1 << PHASE_A);
  IOSET = (1 << PHASE_A);
}

void phase_a_h_off (void)
{
  /* LPC2103 P0.7 --> CPU1 */
  /* set to output */
  IODIR |= (1 << PHASE_A);
  IOCLR = (1 << PHASE_A);
}

void phase_a_l_pwm_on (void)
{
  /* LPC2103 P0.12 (PWM; MAT1.2) --> CPU44 */
  PINSEL0 |= (1 << 25);
}

void phase_a_l_pwm_off (void)
{
  /* set to output */
  IODIR |= (1 << 12);
  IOSET = (1 << 12); /* inverted logic */

  /* LPC2103 P0.12 (PWM; MAT1.2) --> CPU44 */
  PINSEL0 &= ~(1 << 25);
}

void phase_b_h_on (void)
{
  /* LPC2103 P0.8 --> CPU3 */
  /* set to output */
  IODIR |= (1 << PHASE_B);
  IOSET = (1 << PHASE_B);
}

void phase_b_h_off (void)
{
  /* LPC2103 P0.8 --> CPU3 */
  /* set to output */
  IODIR |= (1 << PHASE_B);
  IOCLR = (1 << PHASE_B);
}

void phase_b_l_pwm_on (void)
{
  /* LPC2103 P0.13 (PWM; MAT1.1) --> CPU2 */
  PINSEL0 |= (1 << 27);
}

void phase_b_l_pwm_off (void)
{
  /* set to output */
  IODIR |= (1 << 13);
  IOSET = (1 << 13); /* inverted logic */

  /* LPC2103 P0.13 (PWM; MAT1.1) --> CPU2 */
  PINSEL0 &= ~(1 << 27);
}

void phase_c_h_on (void)
{
  /* LPC2103 P0.9 --> CPU5 */
  /* set to output */
  IODIR |= (1 << PHASE_C);
  IOSET = (1 << PHASE_C);
}

void phase_c_h_off (void)
{
  /* LPC2103 P0.9 --> CPU5 */
  /* set to output */
  IODIR |= (1 << PHASE_C);
  IOCLR = (1 << PHASE_C);
}

void phase_c_l_pwm_on (void)
{
  /* LPC2103 P0.19 (PWM; MAT1.0) --> CPU4 */
  PINSEL1 |= (1 << 7);
}

void phase_c_l_pwm_off (void)
{
  /* set to output */
  IODIR |= (1 << 19);
  IOSET = (1 << 19); /* inverted logic */

  /* LPC2103 P0.19 (PWM; MAT1.0) --> CPU4 */
  PINSEL1 &= ~(1 << 7);
}

void commutation_sector_1 (void)
{
  phase_a_h_off ();
  phase_a_l_pwm_on ();

  phase_b_l_pwm_off ();
  phase_b_h_on ();

  phase_c_h_off ();
  phase_c_l_pwm_off ();
}

void commutation_sector_2 (void)
{
  phase_a_h_off ();
  phase_a_l_pwm_off ();

  phase_b_l_pwm_off ();
  phase_b_h_on ();

  phase_c_h_off ();
  phase_c_l_pwm_on ();
}

void commutation_sector_3 (void)
{
  phase_a_l_pwm_off ();
  phase_a_h_on ();

  phase_b_h_off ();
  phase_b_l_pwm_off ();

  phase_c_h_off ();
  phase_c_l_pwm_on ();
}

void commutation_sector_4 (void)
{
  phase_a_l_pwm_off ();
  phase_a_h_on ();

  phase_b_h_off ();
  phase_b_l_pwm_on ();

  phase_c_h_off ();
  phase_c_l_pwm_off ();
}

void commutation_sector_5 (void)
{
  phase_a_h_off ();
  phase_a_l_pwm_off ();

  phase_b_h_off ();
  phase_b_l_pwm_on ();

  phase_c_l_pwm_off ();
  phase_c_h_on ();
}

void commutation_sector_6 (void)
{
  phase_a_h_off ();
  phase_a_l_pwm_on ();

  phase_b_h_off ();
  phase_b_l_pwm_off ();

  phase_c_l_pwm_off ();
  phase_c_h_on ();
}

void commutation_disable (void)
{
  phase_a_h_off ();
  phase_a_l_pwm_off ();

  phase_b_h_off ();
  phase_b_l_pwm_off ();

  phase_c_h_off ();
  phase_c_l_pwm_off ();
}

void commutation_sector (unsigned int sector)
{
  // commutate
  switch (sector)
  {
    case 1:
    commutation_sector_1 ();
    break;

    case 2:
    commutation_sector_2 ();
    break;

    case 3:
    commutation_sector_3 ();
    break;

    case 4:
    commutation_sector_4 ();
    break;

    case 5:
    commutation_sector_5 ();
    break;

    case 6:
    commutation_sector_6 ();
    break;

    default:
    commutation_disable ();
    break;
  }
}

unsigned int rotor_find_position_sector (void)
{
  unsigned int sector_current[6];
  unsigned int max_current = 0;
  unsigned int max_current_sector = 0;
  unsigned int i;

  motor_set_duty_cycle (1000);
  for (i = 0; i < 6; i++)
  {
    commutation_sector (i + 1); // start energize the sector
    delay_us (50); // wait 50us

    // read the current, 4 samples and average/filter
    sector_current[i] = 0;
    sector_current[i] += (adc_read (CURRENT) / 4);
    sector_current[i] += (adc_read (CURRENT) / 4);
    sector_current[i] += (adc_read (CURRENT) / 4);
    sector_current[i] += (adc_read (CURRENT) / 4);

    commutation_disable ();
    delay_us (50); // wait 50us

    // verify and save the higher current sector
    if (sector_current[i] > max_current)
    {
      max_current = sector_current[i];
      max_current_sector = i + 1;
    }
  }
  commutation_disable ();

  return max_current_sector;
}
