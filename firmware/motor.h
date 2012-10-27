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

void motor_start (void);
void motor_coast (void);
void motor_set_duty_cycle (unsigned int value);
unsigned int motor_get_speed (void);
