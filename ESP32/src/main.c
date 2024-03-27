#include <stdio.h>
#include "utils.h"

void app_main(void)
{

    //reiniciar_esp();

    // Para testar Encoders + Velocidade (Transformação)

    //init_gpio();
    //init_pwm();

    float enc_tick_right = 0;
    // float enc_tick_left = 0;

    //pcnt_unit_handle_t encoder_unit_left = init_encoder(ENC_LEFT);
    pcnt_unit_handle_t encoder_unit_right = init_encoder(ENC_RIGHT);

    while(1){

        enc_tick_right  = pulse_count(encoder_unit_right);

        ESP_LOGI("Teste", "RPM: %.2f | Rad/s: ", enc_tick_right);

        // enc_tick_left  = pulse_count(encoder_unit_left) * K_LEFT;

        // ESP_LOGI("Teste", "Pulse count: %f", enc_tick_left);
    }

}
