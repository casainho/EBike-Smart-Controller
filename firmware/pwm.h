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

#include "main.h"

void pwm_init(void);
void update_duty_cycle(unsigned int value);
void pwm_set_max_current_adc (unsigned int max_current_adc);
