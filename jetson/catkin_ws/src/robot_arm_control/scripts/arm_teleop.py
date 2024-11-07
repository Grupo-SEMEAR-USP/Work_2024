#!/usr/bin/env python3

import rospy
from robot_arm_control.msg import DualServoCommand
from std_msgs.msg import Float32, Int32
from adafruit_servokit import ServoKit
import board
import busio
import time
import getch  

i2c = busio.I2C(board.SCL_1, board.SDA_1) 
kit = ServoKit(channels=16, i2c=i2c, address=0x41)

# Defina os canais dos servos
servo_channel_manip = 0  # Servo do braço/manipulador
servo_channel_gripper = 1  # Servo da garra

# Defina ângulos padrão para a plataforma
plat_angle = 0.0

def move_platform_to_angle(angle):
    global plat_angle
    plat_angle = angle
    rospy.loginfo(f"Movendo plataforma para o ângulo {angle}")
    angle_pub.publish(plat_angle)  # Publica o ângulo da plataforma para o Arduino

def move_manipulator(speed):
    # kit.continuous_servo[servo_channel_manip].throttle = speed
    rospy.loginfo(f"Movendo manipulador no canal {servo_channel_manip} com velocidade {speed}")

def control_gripper(angle):
    # kit.servo[servo_channel_gripper].angle = angle
    rospy.loginfo(f"Movendo garra para o ângulo {angle} no canal {servo_channel_gripper}")

def stop_all_movement():
    """Função para parar todos os movimentos."""
    move_manipulator(0.0)
    rospy.loginfo("Movimento parado em todos os servos.")

def servo_control_node():
    global angle_pub
    rospy.init_node('servo_control_node', anonymous=True)
    
    # Publisher para o ângulo do plat
    angle_pub = rospy.Publisher('/input_data', Float32, queue_size=10)
    
    rospy.loginfo("Iniciando teleop para controle da garra:")
    rospy.loginfo("Controles:\n- 'W' (ângulo 0), 'S' (ângulo 180) para plataforma\n- 'A' (esquerda), 'D' (direita) para manipulador\n- 'F' para fechar garra\n- 'O' para abrir garra\n- 'ESC' para sair")
    
    while not rospy.is_shutdown():
        key = getch.getch()
        
        if key == '\x1b':  # ESC para sair
            rospy.loginfo("Encerrando teleop.")
            stop_all_movement()  # Parar todos os servos ao sair
            rospy.signal_shutdown("Teleop encerrado pelo usuário.")
            break
        elif key == 'a':  # Controle da plataforma
            rospy.loginfo("Comando: Mover plataforma para ângulo 0")
            move_platform_to_angle(0.0)  # Define o ângulo da plataforma para 0
        elif key == 'd':
            rospy.loginfo("Comando: Mover plataforma para ângulo 180")
            move_platform_to_angle(180.0)  # Define o ângulo da plataforma para 180
        elif key == 'w':  # Controle do manipulador
            rospy.loginfo("Comando: Mover manipulador para a esquerda")
            move_manipulator(-0.01)  # Mover manipulador para a esquerda
        elif key == 's':
            rospy.loginfo("Comando: Mover manipulador para a direita")
            move_manipulator(0.18)  # Mover manipulador para a direita
        elif key == 'f':  # Controle da garra
            rospy.loginfo("Comando: Fechar garra")
            control_gripper(130)  # Fechar a garra
        elif key == 'o': 
            rospy.loginfo("Comando: Abrir garra")
            control_gripper(0)  # Abrir a garra
        elif key == ' ': 
            rospy.loginfo("Comando: Parar todos os movimentos")
            stop_all_movement()

if __name__ == '__main__':
    try:
        servo_control_node()
    except rospy.ROSInterruptException:
        pass
