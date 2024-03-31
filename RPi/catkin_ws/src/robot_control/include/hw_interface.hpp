#ifndef ROBOT_HW_INTERFACE_HPP
#define ROBOT_HW_INTERFACE_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "robot_control/velocity_data.h"
#include <cmath>

class RobotHWInterface {
public:
    RobotHWInterface(ros::NodeHandle& nh); // Ajustado para receber NodeHandle por referência
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void publishWheelSpeeds(); // Publicando velocidades do cmd_vel
    void commandTimeoutCallback(const ros::TimerEvent&); // Callback para o timeout
    void updateWheelSpeedForDeceleration(); // Desaceleração
    float mapSpeed(float v_input); // Normalização da velocidade

private:
    ros::NodeHandle nh;
    ros::Publisher velocity_command_pub;
    ros::Subscriber cmd_vel_sub;
    ros::Timer command_timeout_; // Temporizador para o timeout de comandos

    // Variáveis membro para armazenar as velocidades das rodas
    float front_left_wheel_speed = 0.0;
    float front_right_wheel_speed = 0.0;
    float rear_left_wheel_speed = 0.0;
    float rear_right_wheel_speed = 0.0;

    // Parâmetros carregados do arquivo .yaml
    float wheel_radius; // Raio das rodas
    float base_width; // Largura da base do robô
    float wheel_separation_width;
    float wheel_separation_lenght;
    float deceleration_rate; // Taxa de desaceleração
    float max_speed; // Velocidade máxima
    float min_speed; // Velocidade mínima
};

#endif // ROBOT_HW_INTERFACE_HPP
