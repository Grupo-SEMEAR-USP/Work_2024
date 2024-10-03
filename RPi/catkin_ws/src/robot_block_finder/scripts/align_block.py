#!/usr/bin/env python3

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from robot_block_finder.srv import SearchBlock, SearchBlockResponse
from std_msgs.msg import String

has_converged = False
search_active = False  # Controla se a busca está ativa
alignment_pub = None  # Publisher do sinal de alinhamento
target_block_id = None  # ID do bloco que estamos procurando

def start_search_service(req):
    global search_active, has_converged, target_block_id
    search_active = True
    has_converged = False  # Reinicia o estado de convergência
    target_block_id = req.block_id  # Armazena o ID do bloco que queremos buscar
    rospy.loginfo(f"Search for block with ID {target_block_id} initiated.")
    return SearchBlockResponse(success=True, message=f"Search started for block ID {target_block_id}")

def tag_detections_callback(data):
    global has_converged, search_active, alignment_pub, target_block_id

    if not search_active:
        return  # Não faz nada se a busca não foi iniciada

    if len(data.detections) == 0:
        rospy.loginfo("No tags detected.")
        return
    
    if has_converged:
        return

    for detection in data.detections:
        position = detection.pose.pose.pose.position
        tag_id = detection.id[0]

        if tag_id != target_block_id:
            rospy.loginfo(f"Tag detected with ID {tag_id}, but target is {target_block_id}. Ignoring.")
            continue  # Ignora blocos que não são o alvo

        rospy.loginfo(f"Tag detected - ID: {tag_id}")
        rospy.loginfo("Position [x: %f, y: %f, z: %f]", position.x, position.y, position.z)
        
        cmd_vel = Twist()

        if abs(position.y) > 0.05: 
            cmd_vel.linear.x = 0.15 if position.x > 0 else -0.15
            cmd_vel.linear.y = 0.0 
        else:
            cmd_vel.linear.x = 0.0  
            if abs(position.x) > 0.01:  
                cmd_vel.linear.y = 0.15 if position.y > 0 else -0.15
            else:
                cmd_vel.linear.y = 0.0  
                vel_pub.publish(cmd_vel)
                has_converged = True
                rospy.loginfo("Robot has converged.")
                send_found_signal()  # Envia o sinal de alinhamento

        if not has_converged:
            vel_pub.publish(cmd_vel)

def send_found_signal():
    global search_active, alignment_pub
    rospy.loginfo("Block aligned! Sending alignment signal.")
    alignment_pub.publish("aligned")  # Publica o sinal de alinhamento
    search_active = False  # Pausa a busca após encontrar o bloco

def main():
    global vel_pub, alignment_pub

    rospy.init_node('align_block', anonymous=True)

    # Inicializa o Publisher para movimentação do robô
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Publisher para enviar o sinal de alinhamento
    alignment_pub = rospy.Publisher('/alignment_signal', String, queue_size=10)

    # Assinante do tópico de detecção das tags
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_detections_callback)

    # Cria o serviço para iniciar a busca com o ID do bloco
    rospy.Service('/start_search', SearchBlock, start_search_service)

    rospy.loginfo("Ready to start block search service.")

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass