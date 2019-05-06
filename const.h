//
// Created by farwydi on 01.05.2019.
//

#ifndef HEXOPODS__CONST_H_
#define HEXOPODS__CONST_H_

#define GAIT_NONE 0
#define GAIT_METACHRONAL 1
#define GAIT_RIPPLE 2
#define GAIT_TRIPOD 3

// see tower pro 90 spec
// Remember SERVO_MIN -> -90deg, SERVO_MAX -> 90deg
// 4096 * 1ms / 20ms this is the 'minimum' pulse length count (out of 4096)
#define SERVO_MIN 490
// 4096 * 2ms / 20ms this is the 'maximum' pulse length count (out of 4096)
#define SERVO_MAX 2400
#define N_SERVOS 12

#define N_OSC 6 // Number of connected oscillators
#define N_OSC_PLUS_ONE 7
#define N_LEGS 6
#define N_LEGS_PLUS_ONE 7

#endif //HEXOPODS__CONST_H_
