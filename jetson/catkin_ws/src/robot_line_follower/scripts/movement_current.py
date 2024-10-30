#!/usr/bin/env python3

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2  # Biblioteca OpenCV
import numpy as np
import rospy
import time

class MovementController:
    def __init__(self, turn_side, seconds_to_go):
        """
        Inicializa o controlador de movimento com parâmetros personalizados.
        """
        self.velocidade_linear = 0.4  # Velocidade linear
        self.velocidade_angular = 4.0  # Velocidade angular para rotação
        self.tolerancia = 20  # Tolerância para considerar o centróide alinhado
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.stopped = False  # Inicializa o atributo "stopped"
        self.last_known_frame = None  # Armazena o último frame processado
        self.bridge = CvBridge()  # Inicializa o CvBridge corretamente

        # Definir parâmetros de movimento baseados na lógica
        self.turn_side = turn_side
        self.seconds_to_go = seconds_to_go

        # Definir o alvo do centróide (típico para o centro da imagem)
        self.centroide_alvo_x = 320  # Exemplo, dependendo da resolução da câmera
        self.centroide_alvo_y = 240  # Exemplo

    def image_callback(self, data):
        """
        Callback para processar as imagens recebidas da câmera.
        Armazena o último frame e chama o processamento da imagem.
        """
        try:
            # Converte a imagem do formato ROS para o formato OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.last_known_frame = cv_image  # Armazena o último frame

            # Chama o processamento da imagem
            self.process_image(cv_image)
        except CvBridgeError as e:
            rospy.logerr("Erro de CvBridge: %s", e)

    def create_mask(self, frame):
        """
        Cria uma máscara binária para a linha com base no frame fornecido.
        """
        # Converta a imagem para o espaço de cor HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Definir intervalos de cor para segmentação da linha (ajuste conforme necessário)
        lower_color = (0, 0, 0)  # Defina o valor inferior do intervalo de cor
        upper_color = (180, 255, 30)  # Defina o valor superior do intervalo de cor

        # Crie uma máscara binária onde a linha (na cor correta) será destacada
        mask = cv2.inRange(hsv, lower_color, upper_color)

        return mask

    def get_contour_data(self, mask):
        """
        Encontra o contorno da linha e retorna o centróide da linha.
        """
        # Encontrar contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Achar o maior contorno
            c = max(contours, key=cv2.contourArea)

            # Calcular momentos para encontrar o centróide
            M = cv2.moments(c)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return {"x": cx, "y": cy}
            else:
                return None
        return None

    def process_image(self, frame):
        """
        Processa a imagem capturada e verifica a detecção da linha.
        """
        masked_frame = self.create_mask(frame)
        centroide = self.get_contour_data(masked_frame)
        cv2.imshow("Processed Image", masked_frame)
        cv2.waitKey(1)
        
        if centroide:
            cx, cy = centroide["x"], centroide["y"]
            rospy.loginfo(f"Centróide detectado em X: {cx}, Y: {cy}")
            self.adjust_movement(cx, cy)
        else:
            rospy.loginfo("Centróide não detectado. Continuando a girar até encontrá-lo.")
            self.continue_turning()

    def adjust_movement(self, cx, cy):
        """
        Ajusta o movimento do robô com base na posição do centróide detectado.
        """
        # Verifique o desvio do centróide da linha em relação ao alvo
        erro = cx - self.centroide_alvo_x

        # Ajusta a velocidade angular proporcionalmente ao erro
        cmd_vel = Twist()
        cmd_vel.linear.x = self.velocidade_linear
        cmd_vel.angular.z = -float(erro) / 100.0  # Ajuste conforme necessário

        rospy.loginfo(f"Linha detectada - Velocidade Angular: {cmd_vel.angular.z}")
        self.cmd_vel_pub.publish(cmd_vel)

    def continue_turning(self):
        """
        Continua girando o robô até que o centróide seja detectado e centralizado.
        """
        rospy.loginfo("Centróide perdido. Continuando a girar até encontrá-lo.")
        while not rospy.is_shutdown():
            centroide = self.process_last_frame()

            if centroide:
                cx, cy = centroide["x"], centroide["y"]
                if abs(cx - self.centroide_alvo_x) <= self.tolerancia:
                    rospy.loginfo("Centróide detectado e centralizado.")
                    self.stop()
                    break
                else:
                    rospy.loginfo("Ajustando para centralizar o centróide.")
                    self.adjust_movement(cx, cy)
            else:
                cmd_vel = Twist()
                cmd_vel.angular.z = self.velocidade_angular  # Gira até encontrar o centróide
                self.cmd_vel_pub.publish(cmd_vel)

    def walk_forward_for_time(self):
        """
        Faz o robô andar para frente por um tempo determinado.
        """
        rospy.loginfo(f"Andando para frente por {self.seconds_to_go} segundos.")
        cmd_vel = Twist()
        cmd_vel.linear.x = self.velocidade_linear
        self.cmd_vel_pub.publish(cmd_vel)

        # Andar para frente por um tempo definido
        time.sleep(self.seconds_to_go)

        # Parar o robô após o movimento
        self.stop()

    def turn_left_until_centered(self):
        """
        Vira o robô à esquerda até que o centróide esteja centralizado em ambos os eixos (X e Y).
        """
        rospy.loginfo("Virando à esquerda até o centróide estar centralizado.")
        while not rospy.is_shutdown():
            centroide = self.process_last_frame()

            if centroide:
                cx, cy = centroide['x'], centroide['y']
                if abs(cx - self.centroide_alvo_x) <= self.tolerancia and abs(cy - self.centroide_alvo_y) <= self.tolerancia:
                    rospy.loginfo("Centróide centralizado. Parando...")
                    self.stop()
                    break
                else:
                    cmd_vel = Twist()
                    cmd_vel.angular.z = self.velocidade_angular  # Gira à esquerda
                    self.cmd_vel_pub.publish(cmd_vel)
            else:
                rospy.loginfo("Centróide não encontrado. Continuando a girar à esquerda.")

    def turn_right_until_centered(self):
        """
        Vira o robô à direita até que o centróide esteja centralizado em ambos os eixos (X e Y).
        """
        rospy.loginfo("Virando à direita até o centróide estar centralizado.")
        while not rospy.is_shutdown():
            centroide = self.process_last_frame()

            if centroide:
                cx, cy = centroide["x"], centroide["y"]
                if abs(cx - self.centroide_alvo_x) <= self.tolerancia and abs(cy - self.centroide_alvo_y) <= self.tolerancia:
                    rospy.loginfo("Centróide centralizado. Parando...")
                    self.stop()
                    break
                else:
                    cmd_vel = Twist()
                    cmd_vel.angular.z = -self.velocidade_angular  # Gira à direita
                    self.cmd_vel_pub.publish(cmd_vel)
            else:
                cmd_vel = Twist()
                cmd_vel.angular.z = -self.velocidade_angular  # Gira à direita
                self.cmd_vel_pub.publish(cmd_vel)
                rospy.loginfo("Centróide não encontrado. Continuando a girar à direita.")

    def process_last_frame(self):
        """
        Processa o último frame capturado e retorna o centróide da linha.
        """
        if self.last_known_frame is not None:
            masked_frame = self.create_mask(self.last_known_frame)
            return self.get_contour_data(masked_frame)
        else:
            rospy.loginfo("Nenhuma imagem processada ainda.")
            return None

    def stop(self):
        """
        Para o robô.
        """
        rospy.loginfo("Parando o robô.")
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def start_image_processing(self):
        """
        Inicia o processamento de imagens, subscrevendo ao tópico da câmera.
        """
        rospy.Subscriber("/camera/camera/image_raw", Image, self.image_callback)
        rospy.loginfo("Processamento de imagens iniciado.")




#!/usr/bin/env python3

from line_follower import LineFollower
from movement_controller import MovementController
import rospy

def main():
    # Inicialize o nó ROS
    rospy.init_node('robot_controller', anonymous=True)

    # Configurar os parâmetros para o seguidor de linha
    turn_side = 2  # Definir como virar à direita
    turn_at_intersection = 1  # Virar na 1ª interseção
    lost_line_turn = 2  # Virar à direita se perder a linha
    seconds_to_go = 4

    # Criar uma instância do seguidor de linha
    follower = LineFollower(turn_side, turn_at_intersection, lost_line_turn)

    # Iniciar o seguidor de linha
    rospy.loginfo("Iniciando o seguidor de linha...")
    follower.start()

    # Após a finalização do seguidor de linha, realizar a curva à direita
    rospy.loginfo("Seguidor de linha finalizado. Executando a curva para a direita...")

    # Criar uma instância do controlador de movimento para executar a curva à direita
    move = MovementController(turn_side, seconds_to_go)
    
    # Iniciar o processamento de imagens
    move.start_image_processing()
    move.walk_forward_for_time()
    # Executa a virada para a direita até o centróide da linha estar centralizado
    move.turn_right_until_centered()

    # Finalizar o ROS após a execução do movimento
    rospy.signal_shutdown("Movimento finalizado, encerrando o ROS.")

if __name__ == '__main__':
    main()


