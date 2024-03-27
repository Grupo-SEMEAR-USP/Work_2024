#ifndef __UTILS__
#define __UTILS__

#include <stdbool.h>
#include "esp_system.h"

extern bool FLAG_TARGET;

// Target values that will be send to PID 
extern int TARGET_VALUE_L;
extern int TARGET_VALUE_R;

// Values read from encoder
extern int ENCODER_READ_L;
extern int ENCODER_READ_R;

// PWM values that will be send to motors
extern float LEFT_PWM_VALUE;
extern float RIGHT_PWM_VALUE;

extern int PWM_RESOLUTION;

// Values for motor modeling
#define K_LEFT 1;
#define K_RIGHT 0.1475;

#endif