#include "utils.h"

bool FLAG_TARGET = false;

int TARGET_VALUE_L = 0;
int TARGET_VALUE_R = 0;

float ENCODER_READ_L = 0;
float ENCODER_READ_R = 0;

void motor_ctrl() {
    
}

void reiniciar_esp() {

    reset_i2c(I2C_SLAVE_NUM);

    //esp_restart();

}
