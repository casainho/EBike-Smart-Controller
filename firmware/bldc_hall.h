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

void commutation_disable (void);
void commutation_sector (unsigned int sector);
void commutate (void);
unsigned int get_current_sector (void);
float delay_with_current_control (unsigned long us, float current_max);
unsigned int increment_sector (unsigned int sector);
unsigned int decrement_sector (unsigned int sector);
void phase_a_full (void);
void phase_b_full (void);
void phase_c_full (void);
void phase_a_h_on (void);
void phase_b_h_on (void);
void phase_c_h_on (void);
void bldc_align (void);
void phase_a_l_pwm_off (void);
void phase_b_l_pwm_off (void);
void phase_c_l_pwm_off (void);

