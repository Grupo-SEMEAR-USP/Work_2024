#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import numpy as np
from std_msgs.msg import Float32  # Corrigido para importar Float32

def imgmsg_to_numpy(img_msg):
    """
    Converte uma mensagem sensor_msgs/Image para um array NumPy.
    """
    dtype = np.uint8
    img_np = np.frombuffer(img_msg.data, dtype=dtype)
    
    if img_msg.encoding == 'bgr8':
        img_np = img_np.reshape((img_msg.height, img_msg.width, 3))
    elif img_msg.encoding == 'rgb8':
        img_np = img_np.reshape((img_msg.height, img_msg.width, 3))
        img_np = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
    elif img_msg.encoding == 'mono8':
        img_np = img_np.reshape((img_msg.height, img_msg.width))
    else:
        raise NotImplementedError(f"Tipo de codificação {img_msg.encoding} não implementado")
    
    return img_np

def numpy_to_imgmsg(np_img, encoding='bgr8'):
    """
    Converte um array NumPy para uma mensagem sensor_msgs/Image.
    """
    img_msg = Image()
    img_msg.height = np_img.shape[0]
    img_msg.width = np_img.shape[1]
    img_msg.encoding = encoding
    img_msg.is_bigendian = 0
    if encoding == 'bgr8' or encoding == 'rgb8':
        img_msg.step = img_msg.width * 3
    elif encoding == 'mono8':
        img_msg.step = img_msg.width
    else:
        raise NotImplementedError(f"Tipo de codificação {encoding} não implementado")
    img_msg.data = np_img.tobytes()
    return img_msg

class MovementController:
    def __init__(self):
        self.velocidade_linear = 0.2
        self.velocidade_angular = 0.5
        self.left_limit = 170
        self.right_limit = 470
        self.tolerancia_x = 20
        self.tolerancia_y = 40
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # Removido o uso do CvBridge
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.mask_pub = rospy.Publisher("/mask_image", Image, queue_size=1)
        self.feedback_pub = rospy.Publisher("qualquer_coisa_feedback", Float32, queue_size=1)

        self.centroide_alvo_x = None
        self.centroide_alvo_y = None
        self.last_known_frame = None
        self.last_centroid = None

        self.state = "idle"
        self.sequence_complete = 0  # Variável de controle para indicar conclusão da sequência

        # Inicializa as dimensões da imagem como None
        self.image_width = None
        self.image_height = None

        self.use_camera = True

        # Variáveis para controlar o tempo de movimento
        self.move_start_time = None
        self.move_duration = None

        self.turn_start_time = None
        self.turn_duration = None
        self.turn_direction = None  # 'left' ou 'right'

        self.feedback_scheduler = 0.0

        # Variáveis para a sequência de movimentos
        self.sequence_step = 0
        self.movement_sequence = []  # Lista de passos de movimento
        self.current_action = None

        # Configura o timer de controle
        self.control_timer = rospy.Timer(rospy.Duration(1.0/200), self.control_loop)
        self.feedback_timer = rospy.Timer(rospy.Duration(1.0 / 50), self.publish_feedback)

    def publish_feedback(self, event):
        """
        Função chamada a cada 20 ms (50 Hz) para publicar o valor de feedback_scheduler.
        """
        self.feedback_pub.publish(self.feedback_scheduler)

    def set_feedback_scheduler(self, value):
        """
        Função para alterar o valor do feedback_scheduler.
        """
        self.feedback_scheduler = value
        rospy.loginfo(f"Feedback scheduler value set to {value}")
    
    def image_callback(self, data):
        try:
            # Converte a imagem ROS para formato OpenCV
            cv_image = imgmsg_to_numpy(data)
            self.last_known_frame = cv_image

            # Atualiza as dimensões da imagem e o centróide alvo, se necessário
            if self.image_width is None or self.image_height is None:
                height, width = cv_image.shape[:2]
                self.image_width = width
                self.image_height = height
                self.centroide_alvo_x = width // 2
                self.centroide_alvo_y = height // 2
                rospy.loginfo(f"Dimensões da imagem definidas: largura={width}, altura={height}")

            self.process_image(cv_image)
        except Exception as e:
            rospy.logerr(f"Erro ao converter imagem: {e}")

    def process_image(self, frame):
        masked_frame = self.create_mask(frame, self.left_limit, self.right_limit)
        centroide = self.get_contour_data(masked_frame)

        # Atualizar o último centróide
        if centroide:
            self.last_centroid = centroide
        else:
            self.last_centroid = None


    def aply_threshold(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_color = np.array([1, 1, 1]) 
        upper_color = np.array([179, 255, 66])
        mask = cv2.inRange(hsv, lower_color, upper_color)
        return mask

    def create_mask(self, image, left_limit, right_limit):
        h, w = image.shape[:2]  # Altura e largura da imagem
        
        left_mask_points = np.array([(0, 0), (left_limit, 0), (left_limit, h), (0, h)])
        right_mask_points = np.array([(right_limit, 0), (w, 0), (w, h), (right_limit, h)])
        
        mask = np.ones_like(image) * 255  # Inicializa com branco (255)
        
        cv2.fillPoly(mask, [left_mask_points], (0, 0, 0))
        cv2.fillPoly(mask, [right_mask_points], (0, 0, 0))
        
        masked_img = cv2.bitwise_and(image, mask)

        masked_img = self.aply_threshold(masked_img)
        return masked_img

    def get_contour_data(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return {"x": cx, "y": cy}
        return None

    def control_loop(self, event):
        if self.state == "idle":
            if self.sequence_step < len(self.movement_sequence):
                # Obtém a próxima ação
                action = self.movement_sequence[self.sequence_step]
                self.current_action = action
                if action['action'] == 'move_forward':
                    self.walk_forward_for_time(action['duration'])
                elif action['action'] == 'move_back':
                    self.walk_back_for_time(action['duration'])
                elif action['action'] == 'turn':
                    self.turn_for_time_and_center(action['direction'], action['duration'])
                elif action['action'] == 'center_all_sides':  # Nova ação adicionada
                    rospy.loginfo("Iniciando centralização em ambos os eixos (x e y).")
                    self.state = "centering_all_sides"
                self.sequence_step += 1
            else:
                rospy.loginfo("Sequência de movimento concluída.")
                self.sequence_complete += 1
        elif self.state == "moving_forward":
            if rospy.Time.now() - self.move_start_time < rospy.Duration(self.move_duration):
                cmd_vel = Twist()
                cmd_vel.linear.x = self.velocidade_linear
                self.cmd_vel_pub.publish(cmd_vel)
            else:
                self.stop()
                self.state = "idle"
                rospy.loginfo("Movimento para frente concluído.")
                rospy.sleep(0.2)
        elif self.state == "moving_back":
            if rospy.Time.now() - self.move_start_time < rospy.Duration(self.move_duration):
                cmd_vel = Twist()
                cmd_vel.linear.x = -self.velocidade_linear
                self.cmd_vel_pub.publish(cmd_vel)
            else:
                self.stop()
                self.state = "idle"
                rospy.loginfo("Movimento para frente concluído.")
        elif self.state == "turning":
            if rospy.Time.now() - self.turn_start_time < rospy.Duration(self.turn_duration):
                cmd_vel = Twist()
                if self.turn_direction == 'right':
                    cmd_vel.linear.y = -self.velocidade_angular
                elif self.turn_direction == 'left':
                    cmd_vel.linear.y = self.velocidade_angular
                elif self.turn_direction == 'right_z':
                    cmd_vel.angular.z = self.velocidade_angular
                else:
                    cmd_vel.angular.z = -self.velocidade_angular
                self.cmd_vel_pub.publish(cmd_vel)
            else:
                self.stop()
                if self.use_camera:
                    rospy.loginfo("Giro cronometrado concluído. Iniciando centragem na linha.")
                    rospy.sleep(1.0)
                    self.state = "centering"
                else:
                    rospy.loginfo("Não usando a câmera")
                    self.state = "idle"
        elif self.state == "centering":
            if self.last_centroid:
                error = self.last_centroid['x'] - self.centroide_alvo_x
                error_y = self.last_centroid['y'] - self.centroide_alvo_y
                # Calcular a distância em pixels e a direção
                distance = error
                direction = "esquerda" if error < 0 else "direita"
                rospy.loginfo(f"Distância do centróide até o centro: {distance} pixels, Direção: {direction}")

                if abs(error) <= self.tolerancia_x and abs(error_y) <= 100:
                    rospy.loginfo("Centróide centralizado. Parando o robô.")
                    self.stop()
                    self.state = "idle"
                else:
                    cmd_vel = Twist()
                    cmd_vel.linear.y = (-float(error) / 1000.0)  # Ajuste o ganho conforme necessário

                    if(abs(cmd_vel.linear.y) > 0.2):
                        if(cmd_vel.linear.y > 0):
                            cmd_vel.linear.y = 0.2
                        else:
                           cmd_vel.linear.y = -0.2
                        
                    
                    self.cmd_vel_pub.publish(cmd_vel)
                    rospy.loginfo(f"Ajustando para centralizar o centróide. Velocidade angular: {cmd_vel.linear.y}")
            else:
                rospy.loginfo("Faz nada")
                # cmd_vel = Twist()
                # if self.turn_direction == 'right':
                #     cmd_vel.linear.y = self.velocidade_angular
                # else:
                #     cmd_vel.linear.y = -self.velocidade_angular
                # self.cmd_vel_pub.publish(cmd_vel)

        elif self.state == "centering_all_sides":
            if self.last_centroid:
                # Calcule o erro em ambos os eixos (x e y)
                error_x = self.last_centroid['x'] - self.centroide_alvo_x
                error_y = self.last_centroid['y'] - self.centroide_alvo_y

                # Verifica se o centróide está dentro da tolerância em ambos os eixos
                if abs(error_x) <= self.tolerancia_x and abs(error_y) <= self.tolerancia_y:
                    rospy.loginfo("Centróide centralizado em ambos os eixos. Parando o robô.")
                    self.stop()
                    self.state = "idle"
                else:
                    # Ajuste as velocidades para corrigir erros em ambos os eixos
                    cmd_vel = Twist()
                    cmd_vel.linear.y = (-float(error_x) / 1000.0)  # Ajuste para erro em x
                    cmd_vel.linear.x = (-float(error_y) / 1000.0)  # Ajuste para erro em y

                    # Limita as velocidades para evitar movimentos abruptos
                    cmd_vel.linear.y = max(min(cmd_vel.linear.y, 0.2), -0.2)
                    cmd_vel.linear.x = max(min(cmd_vel.linear.x, 0.2), -0.2)

                    self.cmd_vel_pub.publish(cmd_vel)
                    rospy.loginfo(f"Ajustando para centralizar o centróide em ambos os eixos. "
                                  f"Velocidade X: {cmd_vel.linear.x}, Velocidade Y: {cmd_vel.linear.y}")
            else:
                rospy.loginfo("Nenhum centróide detectado.")

        else:
            pass  # Outros estados

    def walk_forward_for_time(self, duration):
        rospy.loginfo(f"Movendo para frente por {duration} segundos.")
        self.state = "moving_forward"
        self.move_duration = duration
        self.move_start_time = rospy.Time.now()
    
    def walk_back_for_time(self, duration):
        rospy.loginfo(f"Movendo para tras por {duration} segundos.")
        self.state = "moving_backward"
        self.move_duration = duration
        self.move_start_time = rospy.Time.now()

    def turn_for_time_and_center(self, direction, duration):
        rospy.loginfo(f"Virando {direction} por {duration} segundos.")
        self.state = "turning"
        self.turn_duration = duration
        self.turn_start_time = rospy.Time.now()
        self.turn_direction = direction

    def stop(self):
        rospy.loginfo("Parando o robô.")
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

    def stop_controller(self):
        """
        Função para parar o controlador. Cancela os timers e para o robô.
        """
        rospy.loginfo("Parando o controlador.")
        self.stop()  # Para o robô
        self.control_timer.shutdown()  # Para o timer principal
        rospy.loginfo("Controlador parado.")

    def stop_feedback(self):
        rospy.loginfo("Parando o feedback do scheduler")
        self.feedback_timer.shutdown()  # Para o timer de feedback

    def execute_movement_sequence(self, sequence):
        self.movement_sequence = sequence
        self.sequence_step = 0
        rospy.loginfo("Iniciando sequência de movimento.")

    def start(self):
        rospy.loginfo("MovementController iniciado.")
        self.feedback_scheduler = 0.0
        # Não é necessário rospy.spin() pois estamos usando timers e callbacks
