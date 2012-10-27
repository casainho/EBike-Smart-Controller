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

/*
 * ADC
 */
// ADC channels
#define VOLTAGE 0
#define CURRENT 1
#define THROTTLE 2

#define BATTERY_VOLTAGE_PER_ADC_STEP 0.035 // 10K/(10K+100K) = 0.091 --> 3.3/0.091 = 36.264 --> 36.264/1023 = 0.035
#define MOTOR_CURRENT_PER_ADC_STEP 0.026 // 6K8/(6K8+3K6) = 0.654 --> sensor outputs 40mV/A --> 0.04/0.654 = 0.026
#define THROTTLE_PER_ADC_STEP 0.035 TODO // 6K8/(6K8+3K6) = 0.654 --> 3.3*0.654 = 36.264 --> 36.264/1023 = 0.035
