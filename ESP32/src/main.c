#include <stdio.h>
#include "utils.h"
#include "i2c_slave.h"

void app_main(void)
{

    // Para testar Encoders + Velocidade (Transformação)

    init_gpio();
    init_pwm();

    pcnt_unit_handle_t encoder_unit_left = init_encoder(ENC_LEFT);
    pcnt_unit_handle_t encoder_unit_right = init_encoder(ENC_RIGHT);

    while(1){

        // Teste 1:
        // update_motor(LEFT, 8192);
        // update_motor(RIGHT, 8192);

        // Teste 2:
        // update_motor(LEFT, 8192);
        // update_motor(RIGHT, 8192);

        // Teste 3:
        // update_motor(LEFT, 8192);
        // update_motor(RIGHT, 8192);

        // Teste 4:
        // update_motor(LEFT, 8192);
        // update_motor(RIGHT, 8192);

        pulse_count(encoder_unit_right);
        pulse_count(encoder_unit_left);
    }

}
