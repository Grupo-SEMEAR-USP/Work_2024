#include "hw_interface.hpp"

RobotHWInterface::RobotHWInterface(ros::NodeHandle& nh) : nh(nh), command_timeout_(nh.createTimer(ros::Duration(0.1), &RobotHWInterface::commandTimeoutCallback, this, true, false)) {
    // Subscreve ao tópico de comandos de movimento
    cmd_vel_sub = nh.subscribe("cmd_vel", 10, &RobotHWInterface::cmdVelCallback, this);

    // Publicador para o comando de velocidade das rodas
    velocity_command_pub = nh.advertise<robot_control::velocity_data>("velocity_command", 10);

    encoder_sub = nh.subscribe("/encoder_data", 10, &RobotHWInterface::encoderCallback, this);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

    // Carregar parâmetros do arquivo .yaml
    nh.getParam("wheel_control/wheel_radius", wheel_radius);
    nh.getParam("wheel_control/wheel_separation_width", wheel_separation_width);
    nh.getParam("wheel_control/wheel_separation_lenght", wheel_separation_lenght);
    nh.getParam("wheel_control/deceleration_rate", deceleration_rate);
    nh.getParam("wheel_control/max_speed", max_speed);
    nh.getParam("wheel_control/min_speed", min_speed);

    base_geometry = (wheel_separation_width + wheel_separation_lenght) / 2;
    base_geometry = 0.35;
}

void RobotHWInterface::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    float vx = msg->linear.x;
    float vy = msg->linear.y;
    float omega = msg->angular.z;

    front_left_wheel_speed = mapSpeed((vx - vy - (omega * base_geometry)) * (1 / wheel_radius));
    front_right_wheel_speed = mapSpeed((vx + vy + (omega * base_geometry)) * (1 / wheel_radius));
    rear_left_wheel_speed = mapSpeed((vx + vy - (omega * base_geometry)) * (1 / wheel_radius));
    rear_right_wheel_speed = mapSpeed((vx - vy + (omega * base_geometry)) * (1 / wheel_radius));

    // Reinicia o temporizador cada vez que um comando é recebido
    command_timeout_.stop();
    command_timeout_.setPeriod(ros::Duration(0.1), true); // Reset com auto-restart
    command_timeout_.start();
}

float RobotHWInterface::mapSpeed(float v_input) {
    return std::min(std::max(v_input, min_speed), max_speed);
}

void RobotHWInterface::publishWheelSpeeds() {
    robot_control::velocity_data msg;

    msg.front_left_wheel = front_left_wheel_speed;
    msg.front_right_wheel = front_right_wheel_speed;
    msg.rear_left_wheel = rear_left_wheel_speed;
    msg.rear_right_wheel = rear_right_wheel_speed;

    velocity_command_pub.publish(msg);
}

void RobotHWInterface::commandTimeoutCallback(const ros::TimerEvent&) {
    updateWheelSpeedForDeceleration();
}

void RobotHWInterface::updateWheelSpeedForDeceleration() {
    // Desacelera cada roda gradualmente até zero
    if (std::abs(front_left_wheel_speed) > deceleration_rate) front_left_wheel_speed -= deceleration_rate * (front_left_wheel_speed > 0 ? 1 : -1);
    else front_left_wheel_speed = 0;

    if (std::abs(front_right_wheel_speed) > deceleration_rate) front_right_wheel_speed -= deceleration_rate * (front_right_wheel_speed > 0 ? 1 : -1);
    else front_right_wheel_speed = 0;

    if (std::abs(rear_left_wheel_speed) > deceleration_rate) rear_left_wheel_speed -= deceleration_rate * (rear_left_wheel_speed > 0 ? 1 : -1);
    else rear_left_wheel_speed = 0;

    if (std::abs(rear_right_wheel_speed) > deceleration_rate) rear_right_wheel_speed -= deceleration_rate * (rear_right_wheel_speed > 0 ? 1 : -1);
    else rear_right_wheel_speed = 0;

    // Verifica se todas as velocidades chegaram a zero, se não, continua desacelerando
    if (front_left_wheel_speed != 0 || front_right_wheel_speed != 0 || rear_left_wheel_speed != 0 || rear_right_wheel_speed != 0) {
        command_timeout_.stop();
        command_timeout_.setPeriod(ros::Duration(0.05), true); // Use um intervalo mais curto para desaceleração suave
        command_timeout_.start();
    }
}

void RobotHWInterface::encoderCallback(const robot_control::encoder_data::ConstPtr& msg) {
    double vel_linearx = (msg->front_right_encoder_data + msg->front_left_encoder_data + msg->rear_right_encoder_data + msg->rear_left_encoder_data) * (wheel_radius / 4.0);
    double vel_lineary = (msg->front_right_encoder_data - msg->front_left_encoder_data - msg->rear_right_encoder_data + msg->rear_left_encoder_data) * (wheel_radius / 4.0);
    double vel_angular_z = (msg->front_right_encoder_data - msg->front_left_encoder_data + msg->rear_right_encoder_data - msg->rear_left_encoder_data) * (wheel_radius / (4.0 * base_geometry));

    std::cout << "Velocidade Linear X: " << vel_linearx << ", Velocidade Linear Y: " << vel_lineary << ", Velocidade Angular Z: " << vel_angular_z << std::endl;

    // aplicar lógica para mandar para a odometria

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "hw_interface");
    ros::NodeHandle nh; // Cria um NodeHandle

    RobotHWInterface controller(nh); // Passa o NodeHandle como argumento

    ros::Rate rate(HW_IF_UPDATE_FREQ);
    ros::AsyncSpinner spinner(4);
    spinner.start();

    while (ros::ok())
    {
        controller.publishWheelSpeeds();
        rate.sleep();
    }

    return 0;
}