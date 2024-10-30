#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32  

# Velocidade linear fixa
LINEAR_SPEED = 0.4
# Constante proporcional fixa para velocidade angular
KP_BASE = 1.5 / 100
# Área mínima para rastrear contornos
MIN_AREA_TRACK = 1100
# Percentagem da largura da imagem para detectar bifurcações
DETECTION_REGION_PERCENTAGE = 0.15
# Tamanho da média móvel
MOVING_AVERAGE_SIZE = 7
# Proporção mínima de pixels brancos nas áreas laterais (mínimo para começar a contar)
MIN_LINE_PROPORTION = 0.3
# Persistência mínima da linha nas áreas laterais (em frames consecutivos)
MIN_PERSISTENCE_FRAMES = 3

class LineFollower:
    def __init__(self, turn_side, turn_at_intersection, lost_line_turn):
        """
        Inicializa o seguidor de linha com os parâmetros necessários:
        - turn_side: Lado para virar (1 para esquerda, 2 para direita)
        - turn_at_intersection: Qual intersecção realizar a virada (1ª, 2ª, 3ª...)
        - lost_line_turn: Lado para virar se a linha for perdida (0 para frente, 1 para esquerda, 2 para direita)
        """
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/camera/image_raw", Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.mask_pub = rospy.Publisher("/mask_image", Image, queue_size=1)
        self.feedback_pub = rospy.Publisher("qualquer_coisa_feedback", Float32, queue_size=1)
        
        # Parâmetros fornecidos como argumento
        self.turn_side = turn_side  # 1 para esquerda, 2 para direita
        self.turn_at_intersection = turn_at_intersection
        self.lost_line_turn = lost_line_turn
        
        # Contador de intersecções
        self.intersection_count = 0
        self.last_known_line_position = None
        self.search_mode = False
        self.intersection_left_count = 0
        self.intersection_right_count = 0
        self.left_buffer = []
        self.right_buffer = []
        self.left_detected = False
        self.right_detected = False
        self.left_persistence = 0
        self.right_persistence = 0
        self.stopped = False  # Flag para indicar quando parar o seguidor de linha

    def image_callback(self, data):
        if self.stopped:
            return  # Não faz mais nada se o seguidor de linha já parou

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.process_image(cv_image)
        except CvBridgeError as e:
            rospy.logerr("Erro de CvBridge: %s", e)

    def process_image(self, current_frame):
        masked_frame = create_mask(current_frame)
        hsv_image = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([0, 0, 38])  # Ajuste esses valores conforme necessário
        upper_blue = np.array([0, 0, 51])
        blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

        line = self.get_contour_data(blue_mask)
        cmd_vel = Twist()

        if line:
            self.search_mode = False
            self.last_known_line_position = line
            x, y = line['x'], line['y']
            _, width = masked_frame.shape[:2]
            error = x - width // 2

            cmd_vel.linear.x = LINEAR_SPEED
            cmd_vel.angular.z = float(error) * -KP_BASE

            # Informações sobre a linha detectada
            rospy.loginfo(f"Linha detectada - Velocidade Angular: {cmd_vel.angular.z:.2f}")
            rospy.loginfo(f"Interseções detectadas - Esquerda: {self.intersection_left_count}, Direita: {self.intersection_right_count}")
            
            cv2.circle(masked_frame, (x, y), 5, (0, 255, 0), -1)
            self.check_for_intersections(blue_mask, masked_frame, cmd_vel)
        else:
            self.search_mode = True
            cmd_vel.linear.x = 0.0
            if self.lost_line_turn == 0:
                cmd_vel.linear.x = 0.4
                cmd_vel.angular.z = 0
                rospy.loginfo("Linha não detectada - Andando para a FRENTE no modo de busca")
            elif self.lost_line_turn == 1:
                cmd_vel.angular.z = 2.5
                rospy.loginfo("Linha não detectada - Virando para ESQUERDA no modo de busca")
            elif self.lost_line_turn == 2:
                cmd_vel.angular.z = -2.5
                rospy.loginfo("Linha não detectada - Virando para DIREITA no modo de busca")

            # Informações de busca
            rospy.loginfo(f"Velocidade Angular no Modo de Busca: {cmd_vel.angular.z:.2f}")
            rospy.loginfo(f"Interseções detectadas - Esquerda: {self.intersection_left_count}, Direita: {self.intersection_right_count}")

        try:
            mask_image_msg = self.bridge.cv2_to_imgmsg(masked_frame, encoding="bgr8")
            self.mask_pub.publish(mask_image_msg)
        except CvBridgeError as e:
            rospy.logerr("Falha ao converter e publicar a imagem da máscara: %s", e)

        cv2.imshow("Processed Image", masked_frame)
        cv2.waitKey(1)
        self.cmd_vel_pub.publish(cmd_vel)

    def check_for_intersections(self, mask, frame, cmd_vel):
        """
        Verifica intersecções à esquerda ou à direita, considerando a presença da linha apenas 
        após ela passar da metade inferior da região lateral.
        """
        h, w = mask.shape
        left_region = mask[:, :int(w * DETECTION_REGION_PERCENTAGE)]
        right_region = mask[:, int(w * (1 - DETECTION_REGION_PERCENTAGE)):]

        # Dividir as regiões laterais em metades (superior e inferior)
        left_lower_half = left_region[int(h / 2):, :]
        right_lower_half = right_region[int(h / 2):, :]

        # Mostrar as regiões no frame para ajuste visual
        cv2.rectangle(frame, (0, int(h / 2)), (int(w * DETECTION_REGION_PERCENTAGE), h), (255, 0, 0), 2)
        cv2.rectangle(frame, (int(w * (1 - DETECTION_REGION_PERCENTAGE)), int(h / 2)), (w, h), (0, 0, 255), 2)

        # Contagem de pixels brancos na metade inferior das regiões laterais (presença de linha)
        left_pixels = cv2.countNonZero(left_lower_half)
        right_pixels = cv2.countNonZero(right_lower_half)
        total_left_pixels = left_lower_half.shape[0] * left_lower_half.shape[1]
        total_right_pixels = right_lower_half.shape[0] * right_lower_half.shape[1]

        # Proporção de pixels brancos nas áreas laterais (metade inferior)
        left_proportion = left_pixels / total_left_pixels
        right_proportion = right_pixels / total_right_pixels

        # Verificar se a linha na esquerda passou do centro e está suficientemente presente
        if left_proportion > MIN_LINE_PROPORTION:
            self.left_persistence += 1
        else:
            self.left_persistence = 0

        if self.left_persistence >= MIN_PERSISTENCE_FRAMES:
            if not self.left_detected:
                self.intersection_left_count += 1
                rospy.loginfo("Interseção detectada à esquerda!")
                self.left_detected = True  # Marcar como detectado para evitar contagens duplicadas
                self.intersection_count += 1  # Contar a intersecção

                # Verificar se esta é a intersecção na qual precisamos virar
                if self.intersection_left_count == self.turn_at_intersection and self.turn_side == 1:
                    # Parar o robô sem encerrar o ROS
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd_vel)
                    rospy.loginfo("Virando à esquerda e parando...")
                    self.stopped = True  # Define o flag de parada
        else:
            self.left_detected = False

        # Verificar se a linha na direita passou do centro e está suficientemente presente
        if right_proportion > MIN_LINE_PROPORTION:
            self.right_persistence += 1
        else:
            self.right_persistence = 0

        if self.right_persistence >= MIN_PERSISTENCE_FRAMES:
            if not self.right_detected:
                self.intersection_right_count += 1
                rospy.loginfo("Interseção detectada à direita!")
                self.right_detected = True  # Marcar como detectado para evitar contagens duplicadas
                self.intersection_count += 1  # Contar a intersecção

                # Verificar se esta é a interseção na qual precisamos virar
                if self.intersection_right_count == self.turn_at_intersection and self.turn_side == 2:
                    # Parar o robô sem encerrar o ROS
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd_vel)
                    rospy.loginfo("Virando à direita e parando...")
                    self.stopped = True  # Define o flag de parada
        else:
            self.right_detected = False

    def get_contour_data(self, mask):
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area > MIN_AREA_TRACK:
                M = cv2.moments(largest_contour)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                return {'x': cx, 'y': cy}
        return None

    def start(self):
        """
        Método para iniciar o processamento das imagens e controle de movimento.
        Controla a taxa de processamento (10Hz) e permite verificar continuamente o estado do robô.
        """
        rate = rospy.Rate(50)  # Define a taxa de loop para 10Hz
        feedback_schduler = 0.0
        while not rospy.is_shutdown() and not self.stopped:
            self.feedback_pub.publish(feedback_schduler)
            # O ROS processa as mensagens recebidas, mas sem a necessidade de 'spinOnce'
            rate.sleep()  # Espera para manter a taxa de 10Hz

    def trade_stopped(self):
        self.stopped = False

def create_mask(image):
    """
    Cria uma máscara para a parte inferior da imagem.
    """
    h, w = image.shape[:2]
    tmask_points = np.array([(0, 0), (w, 0), (w, h), (0, h)])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, [tmask_points], (255, 255, 255))
    masked_img = cv2.bitwise_and(image, mask)
    return masked_img
