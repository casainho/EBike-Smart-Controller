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

#define HALL_SENSORS_MASK ((1<<6) | (1<<4) | (1<<2)) // P0.2, P0.4, P0.6

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

void commutation (void)
{
  unsigned int switch_sequence;

  switch_sequence = (IOPIN & HALL_SENSORS_MASK); // mask other pins

  switch (switch_sequence)
  {
    /*
     * P0.2 -- phase_a
     * P0.4 -- phase_b
     * P0.6 -- phase_c
     *
     */

    /*
     *   c b a
     *   0010000 = 16
     */
    case 16:
    commutation_sector_1 ();
    break;

    /*
     *   c b a
     *   1010000 = 80
     */
    case 80:
    commutation_sector_2 ();
    break;

    /*
     *   c b a
     *   1000000 = 64
     */
    case 64:
    commutation_sector_3 ();
    break;

    /*
     *   c b a
     *   1000100 = 68
     */
    case 68:
    commutation_sector_4 ();
    break;

    /*
     *   c b a
     *   0000100 = 4
     */
    case 4:
    commutation_sector_5 ();
    break;

    /*
     *   c b a
     *   0010100 = 20
     */
    case 20:
    commutation_sector_6 ();
    break;

    default:
    commutation_disable ();
    break;
  }
}
