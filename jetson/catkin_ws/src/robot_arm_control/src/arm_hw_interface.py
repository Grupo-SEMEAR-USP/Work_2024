#!/usr/bin/env python3

import rospy
from robot_arm_control.msg import DualServoCommand
from std_msgs.msg import Float32, Int32
from adafruit_servokit import ServoKit
import board
import busio
import time

i2c = busio.I2C(board.SCL_1, board.SDA_1)  # Verifique se `SCL_1` e `SDA_1` são os pinos corretos para o Jetson
kit = ServoKit(channels=16, i2c=i2c, address=0x41)

# Defina os canais dos servos
servo_channel_plat = 0  # Servo da base
servo_channel_manip = 1  # Servo do braço/manipulador
servo_channel_gripper = 2  # Servo da garra

feedback_pub_dual_servo = None
feedback_pub_gripper = None

def set_servo_speed(channel, speed):
    """Define a velocidade do servo contínuo usando throttle"""
    kit.continuous_servo[channel].throttle = speed

def set_servo_angle(channel, angle):
    """Define o ângulo do servo normal"""
    kit.servo[channel].angle = angle

def run_servo_for_time(channel, speed, duration):
    """Move o servo contínuo a uma determinada velocidade por um tempo específico"""
    rospy.loginfo("Rodando o servo no canal %d com velocidade %f por %f segundos", channel, speed, duration)
    set_servo_speed(channel, speed)
    try:
        time.sleep(duration)
        set_servo_speed(channel, 0.0)  # Para o servo após a duração
        rospy.loginfo("Servo no canal %d parado\n", channel)
        return 1  # Sucesso
    except Exception as e:
        rospy.logerr("Erro ao mover o servo no canal %d: %s", channel, str(e))
        return 0  # Insucesso

def gripper_callback(data):
    """Callback para controle da garra (servo normal)"""
    angle = data.data  
    rospy.loginfo("Movendo gripper para o ângulo %f", angle)
    try:
        set_servo_angle(servo_channel_gripper, angle)  # Define o ângulo do servo da garra
        feedback_pub_gripper.publish(1)  # Sucesso
    except Exception as e:
        rospy.logerr("Erro ao mover o gripper: %s", str(e))
        feedback_pub_gripper.publish(0)  # Insucesso

def dual_servo_callback(data):
    """Callback para controle dos servos contínuos (plat e manip)"""
    plat_success = run_servo_for_time(servo_channel_plat, data.plat_speed, data.plat_duration)
    rospy.sleep(1)
    
    manip_success = run_servo_for_time(servo_channel_manip, data.manip_speed, data.manip_duration)
    rospy.sleep(1)

    if plat_success == 1 and manip_success == 1:
        feedback_pub_dual_servo.publish(1)  # Sucesso para ambos
        rospy.loginfo("Movimentos de plat e manip foram bem-sucedidos.")
    else:
        feedback_pub_dual_servo.publish(0)  # Insucesso em um ou ambos
        rospy.loginfo("Houve falha no movimento de plat ou manip.")

def servo_control_node():
    """Inicializa o nó ROS para controle dos servos"""
    global feedback_pub_dual_servo, feedback_pub_gripper
    
    rospy.init_node('servo_control_node', anonymous=True)

    feedback_pub_dual_servo = rospy.Publisher("/servo_control/dual_servo/feedback", Int32, queue_size=10)
    feedback_pub_gripper = rospy.Publisher("/servo_control/gripper/feedback", Int32, queue_size=10)

    rospy.Subscriber("/servo_control/dual_servo", DualServoCommand, dual_servo_callback)
    rospy.Subscriber("/servo_control/gripper", Float32, gripper_callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_control_node()
    except rospy.ROSInterruptException:
        pass
