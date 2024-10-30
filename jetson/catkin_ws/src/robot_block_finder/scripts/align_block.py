#!/usr/bin/env python3

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range 
from std_msgs.msg import Int32, String  # Para receber o comando de ativação da busca

has_converged = False
search_active = False
alignment_pub = None
target_block_id = None

ultrasonic_left = None
ultrasonic_right = None
ultrasonic_back = None
block_counter = 0
ultrasonic_max_dist = 0.06

def start_search_callback(msg):
    global search_active, has_converged, target_block_id
    search_active = True
    has_converged = False
    target_block_id = int(msg.data)
    rospy.loginfo(f"Buscando pelo bloco de ID {target_block_id} iniciado.")

def tag_detections_callback(data):
    global has_converged, search_active, alignment_pub, target_block_id, block_counter

    if not search_active:
        return

    # Reinicializa o contador de blocos detectados a cada chamada
    block_counter = 0

    if len(data.detections) == 0:
        rospy.loginfo("Nenhuma tag detectada.")
        alignment_pub.publish(0)
        stop_robot()  # Certifique-se de parar o robô se não houver tags
        return
    
    if has_converged:
        return

    detected_blocks = []

    # Limpa a lista de blocos detectados a cada chamada
    for detection in data.detections:
        position = detection.pose.pose.pose.position
        tag_id = detection.id[0]
        detected_blocks.append((tag_id, position))

    # Processa os blocos detectados
    for tag_id, position in detected_blocks:
        if tag_id != target_block_id:
            continue

        block_counter += 1

        cmd_vel = Twist()

        # Movimentação no eixo Y para alinhamento
        if abs(position.y) > 0.01:  
            cmd_vel.linear.x = -0.15 if position.y > 0.01 else 0.15  
        else:
            cmd_vel.linear.x = 0  # Para a movimentação no eixo Y

            # Movimentação no eixo X para alinhamento
            if abs(position.x) > 0.01:  
                cmd_vel.angular.z = -0.15 if position.x > 0.01 else 0.15  
            else:
                cmd_vel.angular.z = 0  # Para a movimentação no eixo X

                # Se o robô está alinhado em ambos os eixos
                if not has_converged:
                    rospy.loginfo("Robot is aligned with the block.")
                    send_found_signal()
                    alignment_pub.publish(1)
                    has_converged = True  # Marca o robô como alinhado
                    cmd_vel.linear.x = 0
                    cmd_vel.linear.y = 0
        
        # Publica os comandos de movimento
        vel_pub.publish(cmd_vel)

    # Se nenhum bloco foi detectado nesta chamada
    if block_counter == 0:
        rospy.loginfo("Bloco não encontrado.")
        alignment_pub.publish(0)
        stop_robot()
        search_active = False

def ultrasonic_callback_left(data):
    global ultrasonic_left
    ultrasonic_left = data.range

def ultrasonic_callback_right(data):
    global ultrasonic_right
    ultrasonic_right = data.range

def ultrasonic_callback_back(data):
    global ultrasonic_back
    ultrasonic_back= data.range

def send_found_signal():
    global search_active
    rospy.loginfo("Bloco alinhado!")
    search_active = False

def stop_robot():
    cmd_vel = Twist()
    cmd_vel.linear.x = 0
    cmd_vel.linear.y = 0
    vel_pub.publish(cmd_vel)

def main():
    global vel_pub, alignment_pub

    rospy.init_node('align_block', anonymous=True)

    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    alignment_pub = rospy.Publisher('/block_align/feedback', Int32, queue_size=10)

    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_detections_callback)

    # rospy.Subscriber('/sensor/ir_right', Range, ultrasonic_callback_left)
    # rospy.Subscriber('/sensor/ir_left', Range, ultrasonic_callback_right)
    # rospy.Subscriber('/sensor/ir_back', Range, ultrasonic_callback_back)

    rospy.Subscriber('/block_align', String, start_search_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass