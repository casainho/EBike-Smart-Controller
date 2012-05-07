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

#include "main.h"

#define CHANNEL_0 0
#define CHANNEL_1 1
#define CHANNEL_2 2

void pwm_init(void);
void update_duty_cycle(unsigned int value);
void pwm_on (unsigned char pwm_phase);
void pwm_off (unsigned char pwm_phase);
