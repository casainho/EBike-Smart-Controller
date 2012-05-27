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

#define PHASE_0_ENABLE_PWM /* MAT1.0 */ IOSET = (1<<12);
#define PHASE_1_ENABLE_PWM /* MAT1.1 */ IOSET = (1<<13);
#define PHASE_2_ENABLE_PWM /* MAT1.2 */ IOSET = (1<<19);
#define PHASE_0_DISABLE_PWM /* MAT1.0 */ IOCLR = (1<<12);
#define PHASE_1_DISABLE_PWM /* MAT1.1 */ IOCLR = (1<<13);
#define PHASE_2_DISABLE_PWM /* MAT1.2 */ IOCLR = (1<<19);

#define PHASE_0_ENABLE_OFF IOSET = (1<<0)
#define PHASE_1_ENABLE_OFF IOSET = (1<<1)
#define PHASE_2_ENABLE_OFF IOSET = (1<<2)
#define PHASE_0_DISABLE_OFF IOCLR = (1<<0)
#define PHASE_1_DISABLE_OFF IOCLR = (1<<1)
#define PHASE_2_DISABLE_OFF IOCLR = (1<<2)

#define DEBUG_ON IOSET = (1<<4);
#define DEBUG_OFF IOCLR = (1<<4);

#define DELAY 50

int main (void)
{
  /* Initialize the system */
  system_init();
  timer1_init();
  enableIRQ();
  //pwm_init();
  ios_init();

  while (1)
  {

    /*             0   1                        0   1
    *           ________                      _________
    * PWM:      |||||||||  2              5  ||||||||||
    * Disable:           ____            ____
    *                        |  3   4   |
    * Enable off:             __________
    */

    DEBUG_ON;

    // 0º to 60º
    PHASE_0_ENABLE_PWM;
    PHASE_0_DISABLE_OFF;
    PHASE_1_DISABLE_PWM;
    PHASE_1_ENABLE_OFF;
    PHASE_2_DISABLE_PWM;
    PHASE_2_DISABLE_OFF;
    delay_us(DELAY);
#if 0
    // 60º to 120º
    PHASE_0_ENABLE_PWM;
    PHASE_0_DISABLE_OFF;
    PHASE_1_DISABLE_PWM;
    PHASE_1_DISABLE_OFF;
    PHASE_2_DISABLE_PWM;
    PHASE_2_ENABLE_OFF;
    delay_us(DELAY);

    // 120º to 180º
    PHASE_0_DISABLE_PWM;
    PHASE_0_DISABLE_OFF;
    PHASE_1_ENABLE_PWM;
    PHASE_1_DISABLE_OFF;
    PHASE_2_DISABLE_PWM;
    PHASE_2_ENABLE_OFF;
    delay_us(DELAY);

    // 180º to 240º
    PHASE_0_DISABLE_PWM;
    PHASE_0_ENABLE_OFF;
    PHASE_1_ENABLE_PWM;
    PHASE_1_DISABLE_OFF;
    PHASE_2_DISABLE_PWM;
    PHASE_2_DISABLE_OFF;
    delay_us(DELAY);

    // 240º to 300º
    PHASE_0_DISABLE_PWM;
    PHASE_0_ENABLE_OFF;
    PHASE_1_DISABLE_PWM;
    PHASE_1_DISABLE_OFF;
    PHASE_2_ENABLE_PWM;
    PHASE_2_DISABLE_OFF;
    delay_us(DELAY);

    // 300º to 360º
    PHASE_0_DISABLE_PWM;
    PHASE_0_DISABLE_OFF;
    PHASE_1_DISABLE_PWM;
    PHASE_1_ENABLE_OFF;
    PHASE_2_ENABLE_PWM;
    PHASE_2_DISABLE_OFF;
    delay_us(DELAY);
#endif
    DEBUG_OFF;
  }
}
