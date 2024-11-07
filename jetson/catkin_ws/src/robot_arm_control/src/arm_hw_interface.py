#!/usr/bin/env python3

import rospy
from robot_arm_control.msg import DualServoCommand  # Mensagem customizada
from std_msgs.msg import Float32, Int32

# Publicadores para feedback
feedback_pub_dual_servo = None
feedback_pub_gripper = None

def publish_platform_angle(angle):
    """Publica o ângulo do platform para o Arduino."""
    rospy.loginfo("Publicando ângulo do platform: %f", angle)
    
    platform_pub.publish(angle)

def publish_manipulator_angle(angle):
    """Publica o ângulo do manipulador para o Arduino."""
    rospy.loginfo("Publicando ângulo do manipulador: %f", angle)

    manipulator_pub.publish(angle)

def dual_servo_callback(data):
    """Callback para o comando dos ângulos."""
    publish_platform_angle(data.platform_angle)

    publish_manipulator_angle(data.manipulator_angle)

    feedback_pub_dual_servo.publish(1)  # Sucesso

def gripper_callback(data):
    """Callback para o gripper."""
    rospy.loginfo("Movendo gripper para o ângulo %f", data.data)

    gripper_pub.publish(data.data)

    feedback_pub_gripper.publish(1)  # Sucesso

def angle_control_node():
    global feedback_pub_dual_servo, feedback_pub_gripper, platform_pub, manipulator_pub, gripper_pub
    
    rospy.init_node('angle_control_node', anonymous=True)

    # Inicializando os publicadores
    feedback_pub_dual_servo = rospy.Publisher("/servo_control/dual_servo/feedback", Int32, queue_size=10)
    feedback_pub_gripper = rospy.Publisher("/servo_control/gripper/feedback", Int32, queue_size=10)

    platform_pub = rospy.Publisher("/platform_angle", Float32, queue_size=10)
    manipulator_pub = rospy.Publisher("/manipulator_angle", Float32, queue_size=10)
    gripper_pub = rospy.Publisher("/gripper_angle", Float32, queue_size=10)

    # Assinaturas dos tópicos para o controle de ângulos
    rospy.Subscriber("/servo_control/dual_servo", DualServoCommand, dual_servo_callback)
    rospy.Subscriber("/servo_control/gripper", Float32, gripper_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        angle_control_node()
    except rospy.ROSInterruptException:
        pass
