#ifndef __UTILS__
#define __UTILS__

#include <stdbool.h>
#include "i2c_slave.h"
#include "esp_system.h"

extern bool FLAG_TARGET;

// Target values that will be send to PID 
extern int TARGET_VALUE_L;
extern int TARGET_VALUE_R;

// Values read from encoder
extern float ENCODER_READ_L;
extern float ENCODER_READ_R;

// Values for motor modeling
#define K_LEFT 1;
#define K_RIGHT 0.1475;

void motor_ctrl();

void reiniciar_esp();

#endif