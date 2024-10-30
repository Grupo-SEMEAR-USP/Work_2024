#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

# Velocidade linear fixa
LINEAR_SPEED = 0.4
# Constante proporcional fixa para velocidade angular
KP_BASE = 1.0 / 100
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

class ImageProcessor:
    def __init__(self, turn_side, turn_at_intersection, lost_line_turn):
        """
        Inicializa o seguidor de linha com os parâmetros necessários:
        - turn_side: Lado para virar (1 para esquerda, 2 para direita)
        - turn_at_intersection: Qual intersecção realizar a virada (1°, 2°, 3°...)
        - lost_line_turn: Lado para virar se a linha for perdida (1 para esquerda, 2 para direita)
        """
        rospy.init_node('line_follower', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/camera/image_raw", Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.mask_pub = rospy.Publisher("/mask_image", Image, queue_size=1)
        
        # Parâmetros fornecidos como argumento
        self.turn_side = turn_side  # 1 para esquerda, 2 para direita
        self.turn_at_intersection = turn_at_intersection
        self.lost_line_turn = lost_line_turn
        
        # Contador de intersecções
        self.intersection_count = 0

        self.last_known_line_position = None
        self.search_mode = False
        self.intersection_left_count = 0  # Contador de intersecções à esquerda
        self.intersection_right_count = 0  # Contador de intersecções à direita

        # Buffers de média móvel para detectar intersecções
        self.left_buffer = []
        self.right_buffer = []

        # Flags para contabilizar intersecções apenas uma vez
        self.left_detected = False
        self.right_detected = False

        # Persistência para garantir que a linha esteja presente por vários frames
        self.left_persistence = 0
        self.right_persistence = 0

    def image_callback(self, data):
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

        rospy.loginfo(f"Contador de intersecção Direita: {self.intersection_right_count}")
        rospy.loginfo(f"Contador de intersecção Esquerda: {self.intersection_left_count}")

        line = self.get_contour_data(blue_mask)
        cmd_vel = Twist()

        if line:
            # Linha detectada
            self.search_mode = False
            self.last_known_line_position = line
            x, y = line['x'], line['y']
            _, width = masked_frame.shape[:2]
            error = x - width // 2

            # Definir velocidade linear e angular
            cmd_vel.linear.x = LINEAR_SPEED
            cmd_vel.angular.z = float(error) * -KP_BASE

            # Desenhar o centróide na imagem
            cv2.circle(masked_frame, (x, y), 5, (0, 255, 0), -1)

            # Verificar intersecção usando a presença e ausência de linhas
            self.check_for_intersections(blue_mask, masked_frame, cmd_vel)

            # Imprimir informações no terminal
            rospy.loginfo(f"Velocidade atual - Linear: {cmd_vel.linear.x}, Angular: {cmd_vel.angular.z}")
            rospy.loginfo("Linha detectada.")

        else:
            # Linha não detectada: parar e iniciar a busca
            self.search_mode = True
            cmd_vel.linear.x = 0.0  # Parar movimento linear

            # Se perder a linha, virar para o lado especificado em 'lost_line_turn'
            if self.lost_line_turn == 1:
                cmd_vel.angular.z = 2.5  # Virar à esquerda
            elif self.lost_line_turn == 2:
                cmd_vel.angular.z = -2.5  # Virar à direita

            rospy.loginfo(f"Velocidade atual - Linear: {cmd_vel.linear.x}, Angular: {cmd_vel.angular.z}")
            rospy.loginfo("Linha não detectada. Modo de busca ativado.")

        # Publicar a imagem da máscara (com ou sem centróide)
        try:
            mask_image_msg = self.bridge.cv2_to_imgmsg(masked_frame, encoding="bgr8")
            self.mask_pub.publish(mask_image_msg)
        except CvBridgeError as e:
            rospy.logerr("Falha ao converter e publicar a imagem da máscara: %s", e)

        # Exibir a imagem em uma janela separada usando OpenCV
        cv2.imshow("Processed Image", masked_frame)  # Exibir a imagem
        cv2.waitKey(1)  # Necessário para atualizar a janela da imagem

        # Publicar comando de velocidade
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
                    # Parar o robô e finalizar o programa
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd_vel)
                    rospy.loginfo("Virando à esquerda e finalizando...")
                    rospy.signal_shutdown("Interseção atingida! Programa finalizado.")
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

                # Verificar se esta é a intersecção na qual precisamos virar
                if self.intersection_right_count == self.turn_at_intersection and self.turn_side == 2:
                    # Parar o robô e finalizar o programa
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd_vel)
                    rospy.loginfo("Virando à direita e finalizando...")
                    rospy.signal_shutdown("Interseção atingida! Programa finalizado.")
        else:
            self.right_detected = False

        # Exibir regiões de interesse
        cv2.imshow("Left Lower Half", left_lower_half)
        cv2.imshow("Right Lower Half", right_lower_half)

    def get_contour_data(self, mask):
        """
        Extrai o maior contorno da máscara (provável linha).
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        line = {}
        max_area = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > MIN_AREA_TRACK:
                M = cv2.moments(contour)
                if M['m00'] > 0:
                    x = int(M["m10"] / M["m00"])
                    y = int(M["m01"] / M["m00"])
                    if area > max_area:
                        max_area = area
                        line['x'] = x
                        line['y'] = y

        return line if line else None

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

def main():
    # Exemplo de argumentos: virar à direita na 2ª intersecção, virar à esquerda se perder a linha
    turn_side = 1  # Virar à direita
    turn_at_intersection = 1  # Virar na 2ª intersecção
    lost_line_turn = 1  # Virar à esquerda se perder a linha

    ImageProcessor(turn_side, turn_at_intersection, lost_line_turn)
    rospy.spin()

if __name__ == '__main__':
    main()


#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class MovementController:
    def __init__(self):
        """
        Inicializa a classe MovementController para controlar o movimento do robô.
        """
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def move_forward(self, duration):
        rospy.loginfo(f"Movendo para frente por {duration} segundos")
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.9  # Velocidade linear constante para frente
        cmd_vel.angular.z = 0.0  # Sem movimento angular
        self.cmd_vel_pub.publish(cmd_vel)
        rospy.sleep(duration)
        self.stop()

    def turn_left_after_forward(self, forward_duration, turn_duration):
        self.move_forward(forward_duration)
        rospy.loginfo(f"Virando à esquerda por {turn_duration} segundos")
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 4.0
        self.cmd_vel_pub.publish(cmd_vel)
        rospy.sleep(turn_duration)
        self.stop()

    def turn_right_after_forward(self, forward_duration, turn_duration):
        self.move_forward(forward_duration)
        rospy.loginfo(f"Virando à direita por {turn_duration} segundos")
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = -4.0
        self.cmd_vel_pub.publish(cmd_vel)
        rospy.sleep(turn_duration)
        self.stop()

    def stop(self):
        rospy.loginfo("Parando o robô")
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)


#################################### code.py
#!/usr/bin/env python3

from line_follower import LineFollower
from movement_controller import MovementController
import rospy

def main():
    # Inicialize o nó ROS
    rospy.init_node('robot_controller', anonymous=True)

    # Configurar os parâmetros para o seguidor de linha
    turn_side = 1  # Definir como virar à direita
    turn_at_intersection = 1  # Virar na 1ª interseção
    lost_line_turn = 1  # Virar à direita se perder a linha
    seconds_to_go = 4

    # Criar uma instância do seguidor de linha
    follower = LineFollower(turn_side, turn_at_intersection, lost_line_turn)

    # Iniciar o seguidor de linha
    rospy.loginfo("Iniciando o seguidor de linha...")
    follower.start()

    # Após a finalização do seguidor de linha, realizar a curva à direita
    rospy.loginfo("Seguidor de linha finalizado. Executando a curva para a direita...")

    # Criar uma instância do controlador de movimento para executar a curva à direita
    move = MovementController()

    move.turn_left_after_forward(0.8, 3)

    # Finalizar o ROS após a execução do movimento
    rospy.signal_shutdown("Movimento finalizado, encerrando o ROS.")

if __name__ == '__main__':
    main()
