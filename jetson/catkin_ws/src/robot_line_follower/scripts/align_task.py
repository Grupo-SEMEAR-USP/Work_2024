#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from apri_tag_ros.msg import TagDetectionArray  # Atualize com o pacote correto

class ArucoTracker:
    def __init__(self):
        rospy.init_node('aruco_tracker')

        # Tópicos
        rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        rospy.Subscriber('/tag_detections', TagDetectionArray, self.aruco_callback)  # Assina o tópico para as tags
        rospy.Subscriber('/ultrassonico1', Float32, self.ultrasonic_callback)
        rospy.Subscriber('/advanced_1', String, self.start_callback)

        # Publicador para mover o robô
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Inicializando variáveis
        self.aruco_list = []  # Lista para armazenar os dados dos ArUcos identificados
        self.max_aruco_count = 7
        self.ultrasonic_distance = float('inf')
        self.found_ids = set()  # Armazenar IDs únicos de ArUcos
        self.moving = False
        self.aruco_detected = 0
        self.aruco_original_order = []
        self.stop = False
        self.start_moving = False
        
        # Parametros fixos
        self.vel_right = Twist()
        self.vel_right.angular.z = 0.5  # Movimento para a direita fixo
        self.vel_left = Twist()
        self.vel_left.angular.z = -0.5  # Movimento para a esquerda fixo

    def start_callback(self, msg):
        # Verifica se a mensagem é '1A' para começar a movimentação
        if msg.data == '1A':
            rospy.loginfo("Iniciando movimentação para a direita.")
            self.start_moving = True

    def ultrasonic_callback(self, msg):
        # Atualiza a distância do ultrassom
        self.ultrasonic_distance = msg.data

    def image_callback(self, msg):
        if self.stop or not self.start_moving:
            return  # Se estiver parado ou ainda não tiver começado, não faz nada
        # Aqui, apenas lemos as imagens, mas não fazemos a conversão ou processamento
        # As imagens podem ser usadas para outros propósitos ou log.
        rospy.loginfo("Imagem recebida do tópico da câmera.")

    def aruco_callback(self, msg):
        if self.stop or not self.start_moving:
            return

        # O callback que recebe informações do tópico "tag_detections"
        # Aqui, `msg` contém a lista de detecções de ArUco
        for detection in msg.detections:
            aruco_id = detection.id
            pose = detection.pose.pose  # Pose já deve estar no formato geometry_msgs/Pose

            if aruco_id not in self.found_ids:
                # Marca o ArUco como detectado e salva no vetor
                pose_data = (pose.position.x, pose.position.y, pose.position.z)
                self.aruco_list.append({'id': aruco_id, 'pose': pose_data})
                self.aruco_original_order.append({'id': aruco_id, 'pose': pose_data})
                self.found_ids.add(aruco_id)
                rospy.loginfo(f"ArUco ID {aruco_id} detectado: Pose {pose_data}")
                self.aruco_detected += 1

            # Condição de parada
            if self.aruco_detected >= self.max_aruco_count or self.ultrasonic_distance < 0.05:
                rospy.loginfo("Limite de ArUcos atingido ou obstáculo detectado.")
                self.stop_movement()
                return

        # Continua movendo para a direita enquanto não atingiu o limite
        self.cmd_vel_pub.publish(self.vel_right)

    def stop_movement(self):
        rospy.loginfo("Parando movimentação.")
        self.stop = True
        self.cmd_vel_pub.publish(Twist())  # Publica velocidade zero para parar

        # Ordena o vetor pelo ID do ArUco
        self.aruco_sorted = sorted(self.aruco_list, key=lambda x: x['id'])
        rospy.loginfo("Vetor de ArUcos ordenado por ID.")

        # Inicia o processo de busca
        self.search_aruco()

    def search_aruco(self):
        # Este método implementará a lógica para buscar o ArUco com base nos IDs ordenados e nas posições
        # Busca o ArUco com o menor ID no vetor ordenado
        rospy.loginfo("Iniciando busca do menor ID no vetor ordenado.")
        # Aqui você implementaria a lógica de buscar o ArUco, comparar a posição e se mover

        # Simulação de parada e sucesso
        rospy.sleep(2)
        rospy.loginfo("Aruco encontrado, movimento concluído.")

        # Anda para frente por 2 segundos
        self.move_forward(2)
        self.move_backward(2)

    def move_forward(self, duration):
        rospy.loginfo("Andando para frente.")
        vel_msg = Twist()
        vel_msg.linear.x = 0.5
        self.cmd_vel_pub.publish(vel_msg)
        rospy.sleep(duration)
        self.cmd_vel_pub.publish(Twist())  # Para o movimento

    def move_backward(self, duration):
        rospy.loginfo("Andando para trás.")
        vel_msg = Twist()
        vel_msg.linear.x = -0.5
        self.cmd_vel_pub.publish(vel_msg)
        rospy.sleep(duration)
        self.cmd_vel_pub.publish(Twist())  # Para o movimento

if __name__ == '__main__':
    try:
        tracker = ArucoTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
