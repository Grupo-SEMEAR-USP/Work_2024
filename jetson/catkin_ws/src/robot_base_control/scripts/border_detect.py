#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist  # Para controlar o movimento do robô

class TableEdgeDetector:
    def __init__(self):
        rospy.init_node('table_edge_detector', anonymous=True)

        # Subscritor para iniciar a detecção ao receber uma mensagem
        self.edge_detect_sub = rospy.Subscriber("/edge_detect", String, self.edge_detect_callback)

        # Subscritor de imagem de profundidade
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)

        # Publisher de movimento do robô
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Publisher de feedback de detecção
        self.edge_feedback_pub = rospy.Publisher("/edge_detect/feedback", Int32, queue_size=10)
        
        # Variável para armazenar o comando de velocidade
        self.twist = Twist()
        self.edge_detected = False  # Variável para evitar continuar publicando comandos após detectar a borda
        self.recent_distances = []  # Lista para armazenar as leituras recentes
        self.movement_active = False  # Para controlar quando o movimento deve iniciar
        
        rospy.loginfo("TableEdgeDetector iniciado, aguardando comando de detecção...")

    def edge_detect_callback(self, msg):
        if msg.data == 'start':
            self.movement_active = True
            self.edge_detected = False  # Reseta a detecção da borda
            self.recent_distances = []  # Reseta as leituras de distância
            rospy.loginfo("Comando de detecção de borda recebido, iniciando...")

    def depth_callback(self, data):
        if not self.movement_active:
            return  # Não fazer nada se o movimento não estiver ativo

        if self.edge_detected:
            return  # Se a borda já foi detectada, não fazer nada

        # Converte os dados da imagem de profundidade para um array NumPy
        try:
            height = data.height
            width = data.width

            # Converte a mensagem Image para um array NumPy
            depth_array = np.frombuffer(data.data, dtype=np.uint16).reshape(height, width)

            # Define a região de interesse (ROI) em torno de uma área à esquerda do centro da imagem
            roi_size = 20  # Define o tamanho da ROI (por exemplo, 20x20 pixels)
            center_x = 100  # Ajustado para a esquerda
            center_y = height // 2
            roi_x_start = max(0, center_x - roi_size // 2)
            roi_x_end = min(width, center_x + roi_size // 2)
            roi_y_start = max(0, center_y - roi_size // 2)
            roi_y_end = min(height, center_y + roi_size // 2)
            
            # Extrair a ROI e calcular a distância média
            roi_depth_data = depth_array[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
            mean_distance = np.mean(roi_depth_data) / 1000.0  # Converter para metros

            rospy.loginfo(f"Distância média da ROI: {mean_distance:.2f} metros")

            # Armazena a leitura recente, mantendo apenas as últimas 5 leituras
            self.recent_distances.append(mean_distance)
            if len(self.recent_distances) > 5:
                self.recent_distances.pop(0)  # Remove a leitura mais antiga

            # Verifica se pelo menos 3 leituras consecutivas são maiores que 0.30 metros
            valid_readings = sum(1 for d in self.recent_distances if d > 0.30)
            if valid_readings >= 3:
                rospy.loginfo("Borda da mesa detectada em 3 leituras consecutivas! Parando o robô.")
                self.stop_robot()  # Para o robô
                self.edge_detected = True  # Marca que a borda foi detectada
                self.movement_active = False  # Interrompe o movimento
                self.edge_feedback_pub.publish(1)  # Publica que a detecção foi um sucesso
            else:
                self.move_right()  # Continua movendo o robô para a direita

        except Exception as e:
            rospy.logerr(f"Erro ao processar a imagem de profundidade: {e}")
            self.edge_feedback_pub.publish(0)  # Publica que houve falha na detecção

    def move_right(self):
        # Define a velocidade para mover o robô para a direita (eixo Y)
        self.twist.linear.x = 0.0  # Sem movimento para frente
        # self.twist.linear.y = 0.0  # Movendo-se para a direita
        self.twist.angular.z = 0.35  # Rotação
        self.cmd_vel_pub.publish(self.twist)
        rospy.loginfo("Movendo para a direita...")

    def stop_robot(self):
        # Define a velocidade para zero, parando o robô
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.loginfo("Robô parado.")

if __name__ == '__main__':
    try:
        detector = TableEdgeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
