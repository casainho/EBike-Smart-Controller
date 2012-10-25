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

#define HALL_SENSORS_MASK ((1<<6) | (1<<4) | (1<<2)) // P0.2, P0.4, P0.6

BYTE bSector = 1;       /* sector of rotor position, 1~6 is possible value */
BOOL fDir = FALSE;      /* motor direction variable---CCW direction is default */

//functions to control each of 6 PWM signals
void phase_u_h_on (void)
{
  /* LPC2103 P0.2 --> CPU4 */
  /* set to output */
  IODIR |= (1 << 2);
  IOSET = (1 << 2);
}

void phase_u_h_off (void)
{
  /* LPC2103 P0.2 --> CPU4 */
  /* set to output */
  IODIR |= (1 << 2);
  IOCLR = (1 << 2);
}

void phase_u_l_pwm_off (void)
{
  /* LPC2103 P0.19 (PWM; MAT1.2) --> CPU4 */
  PINSEL1 |= (1 << 6);
}

void phase_u_l_pwm_on (void)
{
  /* set to output */
  IODIR |= (1 << 19);
  IOSET = (1 << 19); /* inverted logic */

  /* LPC2103 P0.19 (PWM; MAT1.2) --> CPU4 */
  PINSEL1 &= ~(1 << 6);
}

void phase_v_h_on (void)
{
  /* LPC2103 P0.1 --> CPU3 */
  /* set to output */
  IODIR |= (1 << 1);
  IOSET = (1 << 1);
}

void phase_v_h_off (void)
{
  /* LPC2103 P0.1 --> CPU3 */
  /* set to output */
  IODIR |= (1 << 1);
  IOCLR = (1 << 1);
}

void phase_v_l_pwm_off (void)
{
  /* LPC2103 P0.13 (PWM; MAT1.1) --> CPU2 */
  PINSEL0 |= (1 << 26);
}

void phase_v_l_pwm_on (void)
{
  /* set to output */
  IODIR |= (1 << 13);
  IOSET = (1 << 13); /* inverted logic */

  /* LPC2103 P0.13 (PWM; MAT1.1) --> CPU2 */
  PINSEL0 &= ~(1 << 26);
}

void phase_w_h_on (void)
{
  /* LPC2103 P0.0 --> CPU1 */
  /* set to output */
  IODIR |= (1 << 0);
  IOSET = (1 << 0);
}

void phase_w_h_off (void)
{
  /* LPC2103 P0.0 --> CPU1 */
  /* set to output */
  IODIR |= (1 << 0);
  IOCLR = (1 << 0);
}

void phase_w_l_pwm_off (void)
{
  /* LPC2103 P0.12 (PWM; MAT1.0) --> CPU44 */
  PINSEL0 |= (1 << 24);
}

void phase_w_l_pwm_on (void)
{
  /* set to output */
  IODIR |= (1 << 12);
  IOSET = (1 << 12); /* inverted logic */

  /* LPC2103 P0.12 (PWM; MAT1.0) --> CPU44 */
  PINSEL0 &= ~(1 << 24);
}

void commutation_sector_1 (void)
{
  phase_u_h_off ();
  phase_u_l_pwm_on ();

  phase_v_l_pwm_off ();
  phase_v_h_on ();

  phase_w_h_off ();
  phase_w_l_pwm_off ();
}

void commutation_sector_2 (void)
{
  phase_u_h_off ();
  phase_u_l_pwm_off ();

  phase_v_l_pwm_off ();
  phase_v_h_on ();

  phase_w_h_off ();
  phase_w_l_pwm_on ();
}

void commutation_sector_3 (void)
{
  phase_u_l_pwm_off ();
  phase_u_h_on ();

  phase_v_h_off ();
  phase_v_l_pwm_off ();

  phase_w_h_off ();
  phase_w_l_pwm_on ();
}

void commutation_sector_4 (void)
{
  phase_u_l_pwm_off ();
  phase_u_h_on ();

  phase_v_h_off ();
  phase_v_l_pwm_on ();

  phase_w_h_off ();
  phase_w_l_pwm_off ();
}

void commutation_sector_5 (void)
{
  phase_u_h_off ();
  phase_u_l_pwm_off ();

  phase_v_h_off ();
  phase_v_l_pwm_on ();

  phase_w_l_pwm_off ();
  phase_w_h_on ();
}

void commutation_sector_6 (void)
{
  phase_u_h_off ();
  phase_u_l_pwm_on ();

  phase_v_h_off ();
  phase_v_l_pwm_off ();

  phase_w_l_pwm_off ();
  phase_w_h_on ();
}

void commutation_disable (void)
{
  phase_u_h_off ();
  phase_u_l_pwm_off ();

  phase_v_h_off ();
  phase_v_l_pwm_off ();

  phase_w_h_off ();
  phase_w_l_pwm_off ();
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
     *   c b a
     *   0000100 = 4
     */
    case 4:
    commutation_sector_1 ();
    break;

    /*
     *   c b a
     *   1000100 = 68
     */
    case 68:
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
     *   1010000 = 80
     */
    case 80:
    commutation_sector_4 ();
    break;

    /*
     *   c b a
     *   0010000 = 16
     */
    case 16:
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
