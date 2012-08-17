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


//functions to control each of 6 PWM signals
  void Enable_PWM3H(void)
{
     	
/* MAT1.2 pin p0.19*/ PINSEL1 &= ~((1<<6) | (1<<7)); PINSEL1 |= (1<<7);

}

  void Disable_PWM3H(void)
{
     	
/* MAT1.2 pin p0.19*/ IOCLR = (1<<19); PINSEL1 &= ~((1<<6) | (1<<7));

}


void PWM3L_OFF(void)
{
     	
/* GPIO pin p0.2 */ PINSEL0 &= ~((1<<4) | (1<<5));
/* set to output */ IODIR |= (1 << 2);
IOCLR = (1 <<2);
}

void PWM3L_ON(void)
{
     	
/* GPIO pin p0.2 */ PINSEL0 &= ~((1<<4) | (1<<5));
/* set to output */ IODIR |= (1 << 2);
IOSET = (1 <<2);
}

void Enable_PWM2H(void)
{
/* MAT1.1 pin p0.13*/ PINSEL0 &= ~((1<<26) | (1<<27)); PINSEL0 |= (1<<27);
}

void Disable_PWM2H(void)
{
/* MAT1.1 pin p0.13*/ IOCLR = (1<<13); PINSEL0 &= ~((1<<26) | (1<<27));
}
 
void PWM2L_OFF(void)
{
/* GPIO pin p0.2 */ PINSEL0 &= ~((1<<2) | (1<<3));
/* set to output */ IODIR |= (1 << 1);
IOCLR = (1 <<1);
	
}
void PWM2L_ON(void)
{
/* GPIO pin p0.2 */ PINSEL0 &= ~((1<<2) | (1<<3));
/* set to output */ IODIR |= (1 << 1);
IOSET = (1 <<1);

}
void Enable_PWM1H(void)
{
   	
/* MAT1.0 pin p0.12*/ PINSEL0 &= ~((1<<24) | (1<<25)); PINSEL0 |= (1<<25);
}

void Disable_PWM1H(void)
{
   	
/* MAT1.0 pin p0.12*/ IOCLR = (1<<12); PINSEL0 &= ~((1<<24) | (1<<25));
}

void PWM1L_OFF(void)
{
/* GPIO pin p0.1 */ PINSEL0 &= ~((1<<0) | (1<<1));
/* set to output */ IODIR |= (1 << 1);
IOCLR = (1 <<0);
}
void PWM1L_ON(void)
{
/* GPIO pin p0.1 */ PINSEL0 &= ~((1<<0) | (1<<1));
/* set to output */ IODIR |= (1 << 1);
IOSET = (1 <<0);
}


void Commutate(BYTE sector)
{
	switch(sector)
	{
	 case 1: //2
			PWM1L_OFF();
			Disable_PWM2H();
			PWM2L_OFF();
			Disable_PWM3H();

			Enable_PWM1H();	
	 		PWM3L_ON();
	 		break;
	 case 6: 
			Disable_PWM1H();
			PWM1L_OFF();
			PWM2L_OFF();
			Disable_PWM3H();

			Enable_PWM2H();
	 		PWM3L_ON();	
	 		break;
	 case 5: 
			Disable_PWM1H();
			PWM2L_OFF();
			PWM3L_OFF();
			Disable_PWM3H();

			Enable_PWM2H();
	 		PWM1L_ON();	
	 		break;
	 case 4: 
			Disable_PWM1H();
			Disable_PWM2H();
			PWM2L_OFF();
			PWM3L_OFF();

	 		Enable_PWM3H();
	 		PWM1L_ON();	
	 		break;
	 case 3: 
			Disable_PWM1H();
			PWM1L_OFF();
			Disable_PWM2H();
			PWM3L_OFF();

			Enable_PWM3H();
	 		PWM2L_ON();
	 		break;	
	 case 2: 
			PWM1L_OFF();
			Disable_PWM2H();
			PWM3L_OFF();
			Disable_PWM3H();

			Enable_PWM1H();
	 		PWM2L_ON();
	 		break;		 			 
	 default:
	 		Disable_PWM1H();
			PWM1L_OFF();
			Disable_PWM2H();
			PWM2L_OFF();
			PWM3L_OFF();
			Disable_PWM3H();
			break;				 				 				 		
	}

}


void Commutation(void)
{
    Commutate(bSector); 

	
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


