#include "pid.h"

const char *TAG_PID = "PID";

pid_ctrl_block_handle_t init_pid(pid_side_t side){

    pid_ctrl_config_t config_pid; 
    pid_ctrl_block_handle_t pid_block; 

    pid_ctrl_parameter_t values_pid = {
      .kd = PID_SIDE_KD(side),
      .kp = PID_SIDE_KP(side),
      .ki = PID_SIDE_KI(side),
      .min_integral = Min_integral,
      .max_integral = Max_integral,
      .min_output = Min_Output,
      .max_output = Max_Output,
      .cal_type = PID_CAL_TYPE_INCREMENTAL,
    };
    config_pid.init_param = values_pid;

    ESP_ERROR_CHECK(pid_new_control_block(&config_pid, &pid_block));
    ESP_ERROR_CHECK(pid_update_parameters(pid_block, &values_pid));
    return pid_block;
}

void PWM_limit(int *PWM){
  if (*PWM > 1023){
    *PWM = 1023;
  }
  else if (*PWM < -1023){
    *PWM = -1023;
  }
}

esp_err_t pid_calculate(pcnt_unit_handle_t upcnt_unit_L, pid_ctrl_block_handle_t pid_block_L, pcnt_unit_handle_t upcnt_unit_R, pid_ctrl_block_handle_t pid_block_R){
  float controll_pid_LEFT = 0;
  float controll_pid_RIGHT = 0;

  //Target values received is represented in RAD/s
  float target_LEFT = 6;
  float target_RIGHT = 6;

   while(!FLAG_TARGET){

    //Global variables
     ENCODER_READ_L = pulse_count(upcnt_unit_L);
     ENCODER_READ_R = pulse_count(upcnt_unit_R);

     ESP_LOGI(TAG_PID, "Encoder ESQUERDA: %d", ENCODER_READ_L);
     ESP_LOGI(TAG_PID, "Encoder DIREITA: %d", ENCODER_READ_R);
     
     RADS_L = ENCODER_READ_L * PID_TICKS_TO_RADS(PID_LEFT);
     RADS_R = ENCODER_READ_R * PID_TICKS_TO_RADS(PID_RIGHT);

     ESP_LOGI(TAG_PID, "Velocidade inicial ESQUERDA: %f", RADS_L);
     ESP_LOGI(TAG_PID, "Velocidade inicial DIREITA: %f", RADS_R);

     float error_motor_LEFT = (target_LEFT - RADS_L);
     float error_motor_RIGHT = (target_RIGHT - RADS_R);
    
     ESP_LOGI(TAG_PID, "Erro ESQUERDA: %f", error_motor_LEFT);
     ESP_LOGI(TAG_PID, "Erro DIREITA: %f", error_motor_RIGHT);

     // Calculate a new PWM Value
     if(pid_compute(pid_block_L, error_motor_LEFT, &controll_pid_LEFT) != ESP_OK){
       ESP_LOGE(TAG_PID, "Error computing PID");
     }
      if((pid_compute(pid_block_R, error_motor_RIGHT, &controll_pid_RIGHT)) != ESP_OK){
        ESP_LOGE(TAG_PID, "Error computing PID");
      }

     ESP_LOGI(TAG_PID, "PWM command ESQUERDA: %f", controll_pid_LEFT);
     ESP_LOGI(TAG_PID, "PWM command DIREITA: %f", controll_pid_RIGHT);

     LEFT_PWM_VALUE += controll_pid_LEFT;
     RIGHT_PWM_VALUE += controll_pid_RIGHT;

    PWM_limit(&LEFT_PWM_VALUE);
    PWM_limit(&RIGHT_PWM_VALUE);
    
    ESP_LOGI(TAG_PID, "PWM DIREITA: %f", RIGHT_PWM_VALUE);
    ESP_LOGI(TAG_PID, "PWM ESQUERDA: %f", LEFT_PWM_VALUE);

     update_motor(LEFT, LEFT_PWM_VALUE);
     update_motor(RIGHT, RIGHT_PWM_VALUE);

     controll_pid_LEFT = 0;
     controll_pid_RIGHT = 0;

     vTaskDelay(PERIOD / portTICK_PERIOD_MS);

   }

  return ESP_OK;
}
  