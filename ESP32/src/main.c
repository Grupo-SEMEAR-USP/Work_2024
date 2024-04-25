#include <stdio.h>
#include "i2c_slave.h"
#include "pid.h"

void app_main(void)
{

    create_tasks();

        init_gpio();
    init_pwm();

    pid_ctrl_block_handle_t pid_block_left = init_pid(PID_LEFT);
    pid_ctrl_block_handle_t pid_block_right = init_pid(PID_RIGHT);
    pcnt_unit_handle_t encoder_unit_left = init_encoder(ENC_LEFT);
    pcnt_unit_handle_t encoder_unit_right = init_encoder(ENC_RIGHT);

    while(1){
        ENCODER_READ_L = pulse_count(encoder_unit_left);
        ENCODER_READ_R = pulse_count(encoder_unit_right);

        vTaskDelay(2 * FREQ_COMMUNICATION / portTICK_PERIOD_MS);
    }

}
