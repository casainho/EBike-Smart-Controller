/*
 * EBike Smart Controller
 *
 * Copyright (C) Jorge Pinto aka Casainho, 2012, 2013.
 *
 *   casainho [at] gmail [dot] com
 *     www.casainho.net
 *
 * Released under the GPL License, Version 3
 */

void motor_set_max_current (float max_current);
unsigned int motor_get_speed (void);
void motor_set_speed (unsigned int speed);
void motor_start (void);
void motor_coast (void);
void motor_set_duty_cycle (unsigned int value);
