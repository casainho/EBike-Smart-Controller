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

void timer0_capture_init (void);
unsigned int get_timer0_count (void);
void timer2_init (void);
void timer2_set_us (unsigned long us);
void timer2_start (void);
void timer2_stop (void);
void timer3_init (void);
void micros10_clear(void);
unsigned int micros10(void);
void delay_us10(unsigned long us);
