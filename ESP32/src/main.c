#include "esp_log.h"
#include "encoder.h"


void app_main(void) {

    // Testes:
    // - Leitura dos encoders (FUNCIONANDO)
    // - PWM (TESTAR)
    // - I2C (TESTAR) 
    //       --> Provavelmente vamos ter que mudar para transmitir 4 valores
    //       --> Estudar como fazer a leitura do buffer (mesmo erro dando na leitura dos encoders, o que será?)

    // Refazer:
    // - PID

    pcnt_unit_handle_t encoder_unit_right = init_encoder(ENC_RIGHT);

    while(1){
        pulse_count(encoder_unit_right);
    }
}
