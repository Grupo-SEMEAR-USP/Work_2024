#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import String
import time

# Constantes
LINEAR_SPEED = 0.2
KP_BASE = 1.5 / 1000
MIN_AREA_TRACK = 100
DETECTION_REGION_PERCENTAGE = 0.10
MOVING_AVERAGE_SIZE = 7
MIN_LINE_PROPORTION = 0.1
MIN_PERSISTENCE_FRAMES = 3
ANGULAR_Z_DURATION = 3  # Tempo em segundos para o movimento em angular.z
FOLLOWER_INTERVAL = 15  # Tempo em segundos para seguir a linha antes de retorno

def imgmsg_to_numpy(img_msg):
    dtype = np.uint8
    img_np = np.frombuffer(img_msg.data, dtype=dtype)
    
    if img_msg.encoding == 'bgr8':
        img_np = img_np.reshape((img_msg.height, img_msg.width, 3))
    elif img_msg.encoding == 'rgb8':
        img_np = img_np.reshape((img_msg.height, img_msg.width, 3))
        img_np = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
    else:
        raise NotImplementedError(f"Tipo de codificação {img_msg.encoding} não implementado")
    
    return img_np

def numpy_to_imgmsg(np_img, encoding='bgr8'):
    img_msg = Image()
    img_msg.height = np_img.shape[0]
    img_msg.width = np_img.shape[1]
    img_msg.encoding = encoding
    img_msg.is_bigendian = 0
    img_msg.step = np_img.shape[1] * 3
    img_msg.data = np_img.tobytes()
    return img_msg

class LineFollower:
    def __init__(self, turn_side, turn_at_intersection, lost_line_turn, side_surpass, intersection_count_delay, depth_analysis_zones):
        # Inicialização existente
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.move_time_pub = rospy.Publisher("/move_time", String, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.mask_pub = rospy.Publisher("/mask_image", Image, queue_size=1)
        self.feedback_pub = rospy.Publisher("qualquer_coisa_feedback", Float32, queue_size=1)
        
        # Atributos iniciais
        self.turn_side = turn_side
        self.turn_at_intersection = turn_at_intersection
        self.lost_line_turn = lost_line_turn
        self.original_lost_line_turn = lost_line_turn  
        self.side_surpass = side_surpass
        
        self.left_limit = 170
        self.right_limit = 470 
        self.intersection_count = 0
        self.last_known_line_position = None
        self.search_mode = False
        self.intersection_left_count = 0
        self.intersection_right_count = 0
        self.stopped = False
        self.obstacle_detected = False
        
        # Adiciona novos atributos de análise de profundidade
        self.depth_analysis_zones = depth_analysis_zones  # Exemplo: [1, 1, 0]
        self.depth_persistence_count = [0, 0, 0]  # Contador de persistência para cada ponto de análise
        self.depth_threshold = 0.20  # Limite de distância em metros (20 cm)
        self.pixels_to_analyze = [(320, 360), (640, 360), (960, 360)]  # Posição dos três pontos -> MUDAR

        # Controle de tempo e estado
        self.following_start_time = None
        self.angular_movement_started = False
        self.timed_rotation_ready = False

        # Tempo de interseção
        self.intersection_count_delay = intersection_count_delay
        self.start_time = time.time()
        
        
    def depth_callback(self, data):
        # Converte os dados da imagem de profundidade para um array NumPy
        try:
            height = data.height
            width = data.width

            # Converte a mensagem Image para um array NumPy
            depth_array = np.frombuffer(data.data, dtype=np.uint16).reshape(height, width)

            # Define tamanho da ROI em volta de cada ponto
            roi_size = 20  # Tamanho da ROI, por exemplo, 20x20 pixels

            # Processa cada ponto (direita, centro, esquerda) em `self.pixels_to_analyze`
            for i, (center_x, center_y) in enumerate(self.pixels_to_analyze):
                if self.depth_analysis_zones[i] == 0:
                    continue  # Ignora se a zona não está ativa

                # Define os limites da ROI ao redor do ponto atual
                roi_x_start = max(0, center_x - roi_size // 2)
                roi_x_end = min(width, center_x + roi_size // 2)
                roi_y_start = max(0, center_y - roi_size // 2)
                roi_y_end = min(height, center_y + roi_size // 2)

                # Extrai a ROI e calcula a distância média
                roi_depth_data = depth_array[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
                mean_distance = np.mean(roi_depth_data) / 1000.0  # Converte para metros

                rospy.loginfo(f"Distância média da ROI no ponto {i} ({center_x}, {center_y}): {mean_distance:.2f} metros")

                # Armazena a leitura recente para persistência
                if mean_distance < self.depth_threshold:
                    self.depth_persistence_count[i] += 1
                else:
                    self.depth_persistence_count[i] = 0

                # Verifica se a persistência foi atingida para acionar o desvio
                if self.depth_persistence_count[i] >= 3:
                    rospy.loginfo(f"Obstáculo detectado em 3 leituras consecutivas no ponto {i}. Iniciando rotina de desvio.")
                    self.obstacle_detected = True
                    self.handle_obstacle_rotation(i)
                    break  # Evita múltiplos desvios simultâneos

        except Exception as e:
            rospy.logerr(f"Erro ao processar a imagem de profundidade: {e}")

    def image_callback(self, data):
        if self.stopped:
            return

        try:
            cv_image = imgmsg_to_numpy(data)
            self.process_image(cv_image)
        except Exception as e:
            rospy.logerr("Erro ao converter imagem ROS para OpenCV: %s", e)

    def process_image(self, current_frame):
        # Aplicar a máscara corretamente
        masked_frame = create_mask(current_frame, self.left_limit, self.right_limit)
        
        # Continue com o processamento da imagem mascarada
        hsv_image = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([1, 1, 1])
        upper_blue = np.array([179, 255, 66])
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
            cmd_vel.linear.y = float(error) * -KP_BASE

            rospy.loginfo(f"Linha detectada - Velocidade Angular: {cmd_vel.linear.y:.2f}")
            rospy.loginfo(f"Interseções detectadas - Esquerda: {self.intersection_left_count}, Direita: {self.intersection_right_count}")

            cv2.circle(masked_frame, (x, y), 5, (0, 255, 0), -1)
            self.check_for_intersections(blue_mask, masked_frame, cmd_vel)
        else:
            self.search_mode = True
            cmd_vel.linear.x = 0.0
            if self.lost_line_turn == 0:
                cmd_vel.linear.x = 0.2
                cmd_vel.linear.y = 0
                rospy.loginfo("Linha não detectada - Andando para a FRENTE no modo de busca")
            elif self.lost_line_turn == 1:
                cmd_vel.linear.y = 2.5
                rospy.loginfo("Linha não detectada - Virando para ESQUERDA no modo de busca")
            elif self.lost_line_turn == 2:
                cmd_vel.linear.y = -2.5
                rospy.loginfo("Linha não detectada - Virando para DIREITA no modo de busca")

            rospy.loginfo(f"Velocidade Angular no Modo de Busca: {cmd_vel.linear.y:.2f}")
            rospy.loginfo(f"Interseções detectadas - Esquerda: {self.intersection_left_count}, Direita: {self.intersection_right_count}")

        self.cmd_vel_pub.publish(cmd_vel)

        if self.obstacle_detected and not self.angular_movement_started:
            self.handle_obstacle_rotation()
        elif (self.following_start_time and 
              (time.time() - self.following_start_time) >= FOLLOWER_INTERVAL and 
              not self.angular_movement_started and 
              self.timed_rotation_ready):
            self.handle_timed_return_rotation()
        
        # try:
        #     mask_image_msg = numpy_to_imgmsg(masked_frame, encoding="bgr8")
        #     self.mask_pub.publish(mask_image_msg)

        #     image_save_path = "/home/rmajetson/saved_images/processed_image.jpg"
        #     cv2.imwrite(image_save_path, masked_frame)
        # except Exception as e:
        #     rospy.logerr("Falha ao converter e publicar a imagem da máscara: %s", e)

    def handle_obstacle_rotation(self, obstacle_position):
        # Define direção do desvio com base no ponto onde o obstáculo foi detectado
        if obstacle_position == 0:  # Obstáculo na direita
            rospy.loginfo("Desviando para a ESQUERDA devido a obstáculo na direita.")
            comando = "esquerda,3.0"
        elif obstacle_position == 2:  # Obstáculo na esquerda
            rospy.loginfo("Desviando para a DIREITA devido a obstáculo na esquerda.")
            comando = "direita,3.0"
        else:  # Obstáculo no centro
            if self.side_surpass == 1:
                rospy.loginfo("Desviando para a DIREITA devido a obstáculo central.")
                comando = "direita,3.0"
            else:
                rospy.loginfo("Desviando para a ESQUERDA devido a obstáculo central.")
                comando = "esquerda,3.0"

        # Publica o comando de desvio
        self.move_time_pub .publish(comando)
        rospy.sleep(ANGULAR_Z_DURATION)
        self.following_start_time = time.time()
        self.angular_movement_started = True
        self.obstacle_detected = False

    def handle_timed_return_rotation(self):
        rospy.loginfo("Tempo do seguidor expirado, retornando à posição inicial com rotação em angular.z.")
    
        if self.side_surpass == 1:
            comando = "esquerda, 3.0"
        elif self.side_surpass == 2:
            comando = "direita, 3.0"

        self.move_time_pub .publish(comando)
        rospy.loginfo(f"Movimento de rotação reverso com angular.z: {comando}")
        rospy.sleep(3.5)
        
        self.lost_line_turn = self.original_lost_line_turn
        self.angular_movement_started = False
        self.following_start_time = None
        self.timed_rotation_ready = False  # Reseta o controle do timed rotation até nova ativação

    def check_for_intersections(self, mask, frame, cmd_vel):
        # Verifica se o tempo de espera para começar a contar as interseções já passou
        if time.time() - self.start_time < self.intersection_count_delay:
            # Se o tempo não passou, pula o resto da função
            return

        h, w = mask.shape
        
        left_region = mask[:, self.left_limit:int(self.left_limit + (w * DETECTION_REGION_PERCENTAGE))]
        right_region = mask[:, int(self.right_limit - (w * DETECTION_REGION_PERCENTAGE)):self.right_limit]

        left_lower_half = left_region[int(h / 2):, :]
        right_lower_half = right_region[int(h / 2):, :]

        cv2.rectangle(frame, (self.left_limit, int(h / 2)), 
                    (int(self.left_limit + (w * DETECTION_REGION_PERCENTAGE)), h), (255, 0, 0), 2)
        cv2.rectangle(frame, (int(self.right_limit - (w * DETECTION_REGION_PERCENTAGE)), int(h / 2)), 
                    (self.right_limit, h), (0, 0, 255), 2)

        left_pixels = cv2.countNonZero(left_lower_half)
        right_pixels = cv2.countNonZero(right_lower_half)
        
        total_left_pixels = left_lower_half.shape[0] * left_lower_half.shape[1]
        total_right_pixels = right_lower_half.shape[0] * right_lower_half.shape[1]

        left_proportion = left_pixels / total_left_pixels
        right_proportion = right_pixels / total_right_pixels

        if left_proportion > MIN_LINE_PROPORTION:
            self.left_persistence += 1
        else:
            self.left_persistence = 0

        if self.left_persistence >= MIN_PERSISTENCE_FRAMES:
            if not self.left_detected:
                self.intersection_left_count += 1
                rospy.loginfo("Interseção detectada à esquerda!")
                self.left_detected = True
                self.intersection_count += 1

                if self.intersection_left_count == self.turn_at_intersection and self.turn_side == 1:
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd_vel)
                    rospy.loginfo("Virando à esquerda e parando.")
                    self.stopped = True
        else:
            self.left_detected = False

        if right_proportion > MIN_LINE_PROPORTION:
            self.right_persistence += 1
        else:
            self.right_persistence = 0

        if self.right_persistence >= MIN_PERSISTENCE_FRAMES:
            if not self.right_detected:
                self.intersection_right_count += 1
                rospy.loginfo("Interseção detectada à direita!")
                self.right_detected = True
                self.intersection_count += 1

                if self.intersection_right_count == self.turn_at_intersection and self.turn_side == 2:
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd_vel)
                    rospy.loginfo("Virando à direita e parando.")
                    self.stopped = True
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
                rospy.loginfo(f"Contorno detectado - Área: {area}, Centro X: {cx}, Y: {cy}")
                return {'x': cx, 'y': cy}
        rospy.loginfo("Nenhum contorno suficientemente grande detectado.")
        return None

    def start(self):
        rate = rospy.Rate(50)
        feedback_scheduler = 0.0
        rospy.loginfo("Iniciando seguidor de linha.")
        while not rospy.is_shutdown() and not self.stopped:
            # Monitora o tempo desde o último movimento rotacional para ativar o `timed rotation`
            if self.angular_movement_started and (time.time() - self.following_start_time) >= ANGULAR_Z_DURATION:
                self.angular_movement_started = False
                self.timed_rotation_ready = True  # Define pronto para o próximo `timed rotation`

            self.feedback_pub.publish(feedback_scheduler)
            rate.sleep()

def create_mask(image, left_limit, right_limit):
    h, w = image.shape[:2]
    left_mask_points = np.array([(0, 0), (left_limit, 0), (left_limit, h), (0, h)])
    right_mask_points = np.array([(right_limit, 0), (w, 0), (w, h), (right_limit, h)])
    
    mask = np.ones_like(image) * 255
    cv2.fillPoly(mask, [left_mask_points], (0, 0, 0))
    cv2.fillPoly(mask, [right_mask_points], (0, 0, 0))
    
    masked_img = cv2.bitwise_and(image, mask)
    return masked_img