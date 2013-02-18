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

#define MOTOR_MAX_CURRENT 4

#define BATTERY_VOLTAGE_PER_ADC_STEP 0.0089f // 10K/(10K+100K) = 0.091 --> 3.3/0.091 = 36.264 --> 36.264/4095 = 0.0089
#define MOTOR_CURRENT_PER_ADC_STEP 0.031f // 6K8/(6K8+3K6) = 0.654 --> sensor outputs 40mV/A --> 0.04/0.654 = 0.026; 3.3V/4095 = 0.0008; 0.0008/0.026 = 0.031
#define MOTOR_CURRENT_ZERO_AMPS_ADC_VALUE 1998 // 1.61V --> 3.3V ==> 4095; 1.61V ==> 1998

// Throttle config
#define THROTTLE_ADC_MAX 3102 // 2.5V (rounded to lower so throttle will always get there)
#define THROTTLE_ADC_MIN 868 // 0.7V (rounded to higher)
#define THROTTLE_ADC_AMPLITUDE (THROTTLE_ADC_MAX - THROTTLE_ADC_MIN)
