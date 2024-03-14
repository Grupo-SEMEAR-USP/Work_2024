#ifndef __UTILS__
#define __UTILS__

#include <stdbool.h>
#include "h_bridge.h"
#include "encoder.h"

extern bool FLAG_TARGET;

extern int TARGET_VALUE_L;
extern int TARGET_VALUE_R;

extern float ENCODER_READ_L;
extern float ENCODER_READ_R;

void motor_ctrl();

#endif