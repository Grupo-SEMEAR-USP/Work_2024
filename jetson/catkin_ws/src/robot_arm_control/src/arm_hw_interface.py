#!/usr/bin/env python3

import rospy
from robot_arm_control.msg import DualServoCommand
from std_msgs.msg import Float32, Int32
# import board
# import busio
# from adafruit_pca9685 import PCA9685
import time

# i2c = busio.I2C(board.SCL, board.SDA)
# pca = PCA9685(i2c)
# pca.frequency = 50  

servo_channel_plat = 0  
servo_channel_manip = 1  
servo_channel_gripper = 2  

feedback_pub_dual_servo = None
feedback_pub_gripper = None

def set_servo_speed(channel, speed):
    pulse = 1500 + (speed * 500)
    # pca.channels[channel].duty_cycle = int(pulse / 20000 * 0xFFFF)

def set_servo_angle(channel, angle):
    pulse = 500 + (angle / 180.0) * 2000
    # pca.channels[channel].duty_cycle = int(pulse / 20000 * 0xFFFF)

def run_servo_for_time(channel, speed, duration):
    rospy.loginfo("Rodando o servo no canal %d com velocidade %f por %f segundos", channel, speed, duration)
    set_servo_speed(channel, speed)
    try:
        time.sleep(duration)
        set_servo_speed(channel, 0.0)
        rospy.loginfo("Servo no canal %d parado\n", channel)
        return 1  # Sucesso
    except Exception as e:
        rospy.logerr("Erro ao mover o servo no canal %d: %s", channel, str(e))
        return 0  # Insucesso

def gripper_callback(data):
    angle = data.data  
    rospy.loginfo("Movendo gripper para o ângulo %f", angle)
    try:
        set_servo_angle(servo_channel_gripper, angle)
        feedback_pub_gripper.publish(1)  # Sucesso
    except Exception as e:
        rospy.logerr("Erro ao mover o gripper: %s", str(e))
        feedback_pub_gripper.publish(0)  # Insucesso

def dual_servo_callback(data):
    plat_success = run_servo_for_time(servo_channel_plat, data.plat_speed, data.plat_duration)
    rospy.sleep(data.plat_duration + 1)
    
    manip_success = run_servo_for_time(servo_channel_manip, data.manip_speed, data.manip_duration)
    rospy.sleep(data.manip_duration + 1)

    if plat_success == 1 and manip_success == 1:
        feedback_pub_dual_servo.publish(1)  # Sucesso para ambos
        rospy.loginfo("Movimentos de plat e manip foram bem-sucedidos.")
    else:
        feedback_pub_dual_servo.publish(0)  # Insucesso em um ou ambos
        rospy.loginfo("Houve falha no movimento de plat ou manip.")

def servo_control_node():
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
