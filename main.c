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
#include "system.h"
#include "isrsupport.h"
#include "timers.h"
#include "pwm.h"
#include "main.h"
#include "ios.h"

BYTE bSector =1;             /* sector of rotor position, 1~6 is possible value */
BOOL fDir = FALSE;           /* motor direction variable---CCW direction is default */ 
BYTE baStartUpTimeTbl[12]= {180,150,130,100,80,60
                            ,50,40,30,20,10,5};                            
WORD wPWMDutyCycle = 0; 		/* Set PWM initial value in freerun stage  */ 
BYTE bFreeRunTimePointer = 0; /* pointer of startup time table */

void CheckZeroCrossing()
{//TODO
}

//running without any sensor feedback
void FreeRun(void)
{
    WORD wTimeCur;

    /* Add pwm bit by bit, add strength and accerlarate */
    wPWMDutyCycle += 10;
    if(wPWMDutyCycle > MAXPWM)
    wPWMDutyCycle = MAXPWM;

  update_duty_cycle(wPWMDutyCycle);

    /* Switch channel */      
    Commutation();

    wTimeCur = Timer1_wReadTimer();

    /* loop for next freerun commutate */
    while((Timer1_wReadTimer()-wTimeCur)< 100*(WORD)baStartUpTimeTbl[bFreeRunTimePointer]) 
	{ 
	// check for bemf zero crossing here
	CheckZeroCrossing();
    }       

    /* Add speed bit by bit */      
    bFreeRunTimePointer++;
    if(bFreeRunTimePointer >11 ) /* Keep constant speeed */
    {
        bFreeRunTimePointer = 11;
    }


}

int main (void)
{
  /* Initialize the system */
  system_init();
  timer0_init();
  enableIRQ();
  pwm_init();
  ios_init();

  //there is .006ms per cycle
  //timer0_set_us (500); //this is 3ms = .006x500
  timer0_set_us (166); //this is 1ms = .006x166

  // Start timer
  timer0_start ();

  // Set duty-cycle
  wPWMDutyCycle = 100;   /* Set PWM initial value in freerun stage   */
  update_duty_cycle(wPWMDutyCycle);


  while (1)
  {
  	FreeRun();
	
  }
}
