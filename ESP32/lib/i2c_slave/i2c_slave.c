#include "i2c_slave.h"
#include "utils.h"

QueueHandle_t i2c_write_queue = NULL; 
const char *TAG = "i2c-teste"; 

esp_err_t i2c_slave_init(void) {
    
    int i2c_slave_port = I2C_SLAVE_NUM;

    i2c_config_t conf_slave = {
        .sda_io_num = I2C_SLAVE_SDA_IO,         
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SLAVE_SCL_IO,          
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .mode = I2C_MODE_SLAVE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = I2C_SLAVE_ADDRESS,
    };

    esp_err_t err = i2c_param_config(i2c_slave_port, &conf_slave);

    if (err != ESP_OK) {
        return err;
    }
    
    return i2c_driver_install(i2c_slave_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
}

void i2c_read_task() {

    int read_value_r = 0;
    int read_value_l = 0;

    uint8_t rx_data[I2C_SLAVE_RX_BUF_LEN];

    int size = i2c_slave_read_buffer(I2C_SLAVE_NUM, rx_data, READ_LEN_VALUE, TIMEOUT_MS / portTICK_PERIOD_MS);

    // "Unpacking" data values (right)
    for(int i = 2; i < 6; i++){
        read_value_r |= (rx_data[i] << 8 * ((READ_LEN_VALUE - 1) - i));
    } 
    
    // "Unpacking" data values (left)
    for(int i = 6; i < 9; i++){
        read_value_l |= (rx_data[i] << 8 * ((READ_LEN_VALUE - 1) - i));
    } 

    if (size > 0) {

        if(TARGET_VALUE_R != read_value_r ||
        TARGET_VALUE_L != read_value_l){
            FLAG_TARGET = true;
        } else {
            FLAG_TARGET = false;
        }

        printf("Read value: %d, %d\n", read_value_r, read_value_l);

        TARGET_VALUE_R = read_value_r;
        TARGET_VALUE_L = read_value_l;

    } else {
        ESP_LOGI(TAG, "Read failed!");
    }

}

void i2c_write_task(int value_r, int value_l) {
    uint8_t tx_data[WRITE_LEN_VALUE];

    // "Packing" values right
    for (int i = 0; i < WRITE_LEN_VALUE/ 2; i++) {
        tx_data[i] = (value_r >> 8 * ((WRITE_LEN_VALUE - 1) - i)) & LAST_BYTE_MASK;
    }

    // "Packing" values left
    for (int i = WRITE_LEN_VALUE / 2; i < WRITE_LEN_VALUE; i++) {
        tx_data[i] = (value_l >> 8 * ((WRITE_LEN_VALUE - 1) - i)) & LAST_BYTE_MASK;
    }

    int size = i2c_slave_write_buffer(I2C_SLAVE_NUM, tx_data, WRITE_LEN_VALUE, TIMEOUT_MS / portTICK_PERIOD_MS);

    if (size > 0) {
        printf("Write value: %d, %d\n", value_r, value_l);
    } else {
        ESP_LOGI(TAG, "Write failed!");
    }
    
}

void i2c_task_com() {
    ESP_ERROR_CHECK(i2c_slave_init());
    i2c_write_queue = xQueueCreate(10, I2C_SLAVE_TX_BUF_LEN);
    int teste = 0;
    while(1){
        vTaskDelay(FREQ_COMMUNICATION / portTICK_PERIOD_MS);
        teste = teste + 1;
        i2c_read_task();
        vTaskDelay(FREQ_COMMUNICATION / portTICK_PERIOD_MS);
        i2c_write_task(teste, 0);
    }
}

void task_motor_control() {

    // init_gpio();
    // init_pwm();

    // pid_ctrl_block_handle_t pid_block_left = init_pid(PID_LEFT);
    // pid_ctrl_block_handle_t pid_block_right = init_pid(PID_RIGHT);
    // pcnt_unit_handle_t encoder_unit_left = init_encoder(ENC_LEFT);
    // pcnt_unit_handle_t encoder_unit_right = init_encoder(ENC_RIGHT);

    while(1){
        // pid_calculate(encoder_unit_left, pid_block_left, encoder_unit_right, pid_block_right);

        // Testing:

        // update_motor(LEFT, 8192);
        // update_motor(RIGHT, 8192);
        // vTaskDelay(2*FREQ_COMMUNICATION / portTICK_PERIOD_MS);
    }
   
}

esp_err_t create_tasks() {

    // Task 1 (core 0): read + write data
    xTaskCreatePinnedToCore(i2c_task_com, "i2c_task_com", 4096, NULL, 5, NULL, 0);

    // Task 2 (core 1): control 
    // xTaskCreatePinnedToCore(task_motor_control, "task_motor_control", 4096, NULL, 1, NULL, 1);

    return ESP_OK;
}

esp_err_t reset_i2c(i2c_port_t i2c_num){
    // Desabilita o driver I2C
    esp_err_t ret = i2c_driver_delete(i2c_num);
    if (ret != ESP_OK) {
        return ret;
    }

    return ret;
}

