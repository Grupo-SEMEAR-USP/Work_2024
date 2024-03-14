#include <stdio.h>
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void restart_task(void *pvParameters) {
    // Atraso de 3 segundos antes de reiniciar
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI("Restart", "Reiniciando ESP32");
    esp_restart();
}

void app_main(void) {
    // Iniciar tarefa para reiniciar após atraso
    xTaskCreate(&restart_task, "restart_task", 2048, NULL, 5, NULL);

    // Seu código principal aqui

    // Testes:
    // - Leitura dos encoders (FUNCIONANDO)
    // - PWM (FUNCIONANDO)

    // - I2C (TESTAR) 
    //       --> Provavelmente vamos ter que mudar para transmitir 4 valores
    //       --> Estudar como fazer a leitura do buffer (mesmo erro dando na leitura dos encoders, o que será?) --> Vou tentar resetar a ESP antes de começar

    // Refazer:
    // - PID
    
    // create_tasks();
}
