#include "esp_log.h"
#include "encoder.h"

void app_main(void) {
    
    // init_gpio();
    // init_pwm();

    pcnt_unit_handle_t encoder_unit_left = init_encoder(ENC_LEFT);
    pcnt_unit_handle_t encoder_unit_right = init_encoder(ENC_RIGHT);

    pulse_count(encoder_unit_left);
    pulse_count(encoder_unit_right);

}