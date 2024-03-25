#ifndef __UTILS__
#define __UTILS__

#include <stdbool.h>
#include "h_bridge.h"
#include "encoder.h"

extern bool FLAG_TARGET;

// Target values that will be send to PID 
extern int TARGET_VALUE_L;
extern int TARGET_VALUE_R;

// Values read from encoder
extern float ENCODER_READ_L;
extern float ENCODER_READ_R;

// PWM values that will be send to motors
extern float LEFT_PWM_VALUE;
extern float RIGHT_PWM_VALUE;

void motor_ctrl();

#endif