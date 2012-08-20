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

extern BYTE bSector;
extern BOOL fDir;

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


void phase_u_h_pwm_on (void)
{
  /* LPC2103 P0.2 --> CPU4 */
  /* set to output */
  IODIR |= (1 << 2);
  IOSET = (1 << 2);
}

void phase_u_h_pwm_off (void)
{
  /* LPC2103 P0.2 --> CPU4 */
  /* set to output */
  IODIR |= (1 << 2);
  IOCLR = (1 << 2);
}

void phase_u_l_pwm_on (void)
{
  /* LPC2103 P0.19 (PWM; MAT1.2) --> CPU4 */
//  PINSEL1 |= (1 << 6);
  /* LPC2103 P0.19 (PWM; MAT1.2) --> CPU4 */
  PINSEL1 &= ~(1 << 6);
  /* set to output */
  IODIR |= (1 << 19);
  IOCLR = (1 << 19); /* inverted logic */
}

void phase_u_l_pwm_off (void)
{
  /* LPC2103 P0.19 (PWM; MAT1.2) --> CPU4 */
  PINSEL1 &= ~(1 << 6);
  /* set to output */
  IODIR |= (1 << 19);
  IOSET = (1 << 19); /* inverted logic */
}

void phase_v_h_pwm_on (void)
{
  /* LPC2103 P0.1 --> CPU3 */
  /* set to output */
  IODIR |= (1 << 1);
  IOSET = (1 << 1);
}

void phase_v_h_pwm_off (void)
{
  /* LPC2103 P0.1 --> CPU3 */
  /* set to output */
  IODIR |= (1 << 1);
  IOCLR = (1 << 1);
}

void phase_v_l_pwm_on (void)
{
  /* LPC2103 P0.13 (PWM; MAT1.1) --> CPU2 */
//  PINSEL0 |= (1 << 26);
  /* set to output */
  IODIR |= (1 << 13);
  IOCLR = (1 << 13); /* inverted logic */
}

void phase_v_l_pwm_off (void)
{
  /* LPC2103 P0.13 (PWM; MAT1.1) --> CPU2 */
  PINSEL0 &= ~(1 << 26);
  /* set to output */
  IODIR |= (1 << 13);
  IOSET = (1 << 13); /* inverted logic */
}

void phase_w_h_pwm_on (void)
{
  /* LPC2103 P0.0 --> CPU1 */
  /* set to output */
  IODIR |= (1 << 0);
  IOSET = (1 << 0);
}

void phase_w_h_pwm_off (void)
{
  /* LPC2103 P0.0 --> CPU1 */
  /* set to output */
  IODIR |= (1 << 0);
  IOCLR = (1 << 0);
}

void phase_w_l_pwm_on (void)
{
  /* LPC2103 P0.12 (PWM; MAT1.0) --> CPU44 */
//  PINSEL0 |= (1 << 24);
  /* set to output */
  IODIR |= (1 << 12);
  IOCLR= (1 << 12); /* inverted logic */
}

void phase_w_l_pwm_off (void)
{
  /* LPC2103 P0.12 (PWM; MAT1.0) --> CPU44 */
  PINSEL0 &= ~(1 << 24);
  /* set to output */
  IODIR |= (1 << 12);
  IOSET = (1 << 12); /* inverted logic */
}

void commutation_sector_1 (void)
{
  phase_u_l_pwm_off ();
  phase_u_h_pwm_on ();

  phase_v_h_pwm_off ();
  phase_v_l_pwm_on ();

  phase_w_h_pwm_off ();
  phase_w_l_pwm_off ();
}

void commutation_sector_2 (void)
{
  phase_u_l_pwm_off ();
  phase_u_h_pwm_on ();

  phase_v_h_pwm_off ();
  phase_v_l_pwm_off ();

  phase_w_h_pwm_off ();
  phase_w_l_pwm_on ();
}

void commutation_sector_3 (void)
{
  phase_u_h_pwm_off ();
  phase_u_l_pwm_off ();

  phase_v_l_pwm_off ();
  phase_v_h_pwm_on ();

  phase_w_h_pwm_off ();
  phase_w_l_pwm_on ();
}

void commutation_sector_4 (void)
{
  phase_u_h_pwm_off ();
  phase_u_l_pwm_on ();

  phase_v_l_pwm_off ();
  phase_v_h_pwm_on ();

  phase_w_h_pwm_off ();
  phase_w_l_pwm_off ();
}

void commutation_sector_5 (void)
{
  phase_u_h_pwm_off ();
  phase_u_l_pwm_on ();

  phase_v_h_pwm_off ();
  phase_v_l_pwm_off ();

  phase_w_l_pwm_off ();
  phase_w_h_pwm_on ();
}

void commutation_sector_6 (void)
{
  phase_u_h_pwm_off ();
  phase_u_l_pwm_off ();

  phase_v_h_pwm_off ();
  phase_v_l_pwm_on ();

  phase_w_l_pwm_off ();
  phase_w_h_pwm_on ();
}

void commutation_disable (void)
{
  phase_u_h_pwm_off ();
  phase_u_l_pwm_off ();

  phase_v_h_pwm_off ();
  phase_v_l_pwm_off ();

  phase_w_h_pwm_off ();
  phase_w_l_pwm_off ();
}

void commutate (BYTE sector)
{
  switch (sector)
  {
    case 1:
    commutation_sector_1 ();
    break;

    case 6:
    commutation_sector_6 ();
    break;

    case 5:
    commutation_sector_5 ();
    break;

    case 4:
    commutation_sector_4 ();
    break;

    case 3:
    commutation_sector_3 ();
    break;

    case 2:
    commutation_sector_2 ();
    break;

    default:
    commutation_disable ();
    break;
  }
}

void Commutation(void)
{
    commutate(bSector);

	
	/* ClockWise rotation */
	if(fDir)
	{
	    bSector--;
	    if(bSector<1) bSector =6;
	}
	    
	else
	{
	    bSector++;
	    if(bSector>6) bSector =1;      
	}
	    
}


