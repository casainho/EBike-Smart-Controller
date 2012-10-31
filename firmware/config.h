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

#define MOTOR_MAX_CURRENT 4

// ADC channels
#define VOLTAGE 0
#define CURRENT 1
#define THROTTLE 2

#define BATTERY_VOLTAGE_PER_ADC_STEP 0.035f // 10K/(10K+100K) = 0.091 --> 3.3/0.091 = 36.264 --> 36.264/1023 = 0.035
#define MOTOR_CURRENT_PER_ADC_STEP 0.026f // 6K8/(6K8+3K6) = 0.654 --> sensor outputs 40mV/A --> 0.04/0.654 = 0.026
#define THROTTLE_PERCENT_PER_ADC_STEP 0.092f // 6K8/(6K8+3K6) = 0.654 --> 5*0.654 = 3.27 --> 3.27/1023 = 0.003 --> 5V -> 100%, 0.003 -> 0.092

//Phace C com PWM e Phase B com Low ligado.
