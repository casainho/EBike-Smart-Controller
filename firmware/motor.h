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
float motor_get_current (void);
void motor_current_control (unsigned int duty_cycle);
void motor_set_current_max (float current_max);
unsigned int motor_is_coast (void);
