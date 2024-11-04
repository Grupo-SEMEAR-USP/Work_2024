#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String, Float32
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Range

class BlockSorter:
    def __init__(self):
        rospy.init_node('block_sorter')

        # Inicialização dos vetores
        self.original_order = []
        self.ordered_ids = []
        self.current_detections = []

        # Variáveis de controle
        self.mapping_done = False
        self.ultrasound_right = float('inf')
        self.ultrasound_left = float('inf')
        self.advanced_1 = 0
        self.is_executing = False
        self.servo_feedback = 0 #Determinar como falso
        self.table_feedback = 0 #Determinar como falso

        # Subscritores
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.aruco = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.aruco_callback)
        self.ultrasound_right_sub = rospy.Subscriber("ultrasound_right", Float32, self.ultrasound_right_callback)
        self.ultrasound_left_sub = rospy.Subscriber("ultrasound_left", Float32, self.ultrasound_left_callback)
        self.advanced_1_sub = rospy.Subscriber("advanced_1", Int32, self.advanced_1_callback)
        self.servo_control_feedback = rospy.Subscriber("servo_control/move_to_pose/feedback", Int32, self.update_servo_feedback)
        self.align_table_feedback = rospy.Subscriber("/align_table/feedback", Int32, self.update_align_table_feedback)

        # Publicadores
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.servo_control_pub = rospy.Publisher("servo_control/move_to_pose", String, queue_size=10)
        self.align_table_pub = rospy.Publisher("/align_table", String, queue_size=10)
        self.pick_block_pub = rospy.Publisher("pick_block", String, queue_size=10)
        self.pub_move_time = rospy.Publisher('/move_time', String, queue_size=10)

        self.rate = rospy.Rate(10)

    # Callback para a imagem da câmera (pode ser utilizado para processamento futuro)
    def image_callback(self, msg):
        pass
    
    def update_servo_feedback(self, data):
        self.servo_feedback = data.data

    def update_align_table_feedback(self, data):
        self.table_feedback = data.data

    # Callback para os marcadores Aruco
    def aruco_callback(self, msg):
        if len(msg.detections) == 0:
            rospy.loginfo(f"Nenhum bloco detectado.")
        for marker in msg.detections:
            if marker.id not in [item['id'] for item in self.original_order]:
                aruco_info = {
                    'id': marker.id[0],
                    'pose': marker.pose.pose.pose.position
                }
                self.original_order.append(aruco_info)
                rospy.loginfo(f"ArUco ID {marker.id} detectado.")
        
        for detection in msg.detections:
            position = detection.pose.pose.pose.position
            tag_id = detection.id[0]
            self.current_detections.append((tag_id, position))

    # Callbacks para os sensores ultrassônicos
    def ultrasound_right_callback(self,data):
        self.ultrasound_right = data/100.0

    def ultrasound_left_callback(self, data):
        self.ultrasound_left = data/100.0

    def advanced_1_callback(self, msg):
        if msg.data == 1 and not self.is_executing:
            rospy.loginfo("Comando de início recebido via advanced_1.")
            self.is_executing = True
            self.execute()

    # Processo de mapeamento dos blocos
    def mapping_process(self):
        rospy.loginfo("Iniciando o mapeamento dos blocos.")
        twist = Twist()
        twist.angular.z = 0.5  # Movimenta para a direita

        while not rospy.is_shutdown():
            if len(self.original_order) >= 7:
                rospy.loginfo("Todos os 7 ArUcos foram mapeados.")
                break
            if self.ultrasound_right < 0.15:
                rospy.loginfo("Limite do ultrassom direito alcançado.")
                break
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.mapping_done = True
        rospy.loginfo("Mapeamento concluído.")

    # Processo de busca por ID específico
    def search_specific_id(self, pos1, pos2, element_id):
        rospy.loginfo(f"Buscando o ArUco ID {element_id}.")
        if pos1 < pos2:
            self.search_right(element_id)
        elif pos1 > pos2:
            self.search_left(element_id)
        else:
            rospy.loginfo("Posições iguais. Nenhuma ação necessária.")

    # Busca para a esquerda
    def search_left(self, element_id):
        twist = Twist()
        twist.angular.z = 0.5  # Movimenta para a esquerda

        while not rospy.is_shutdown():
            if any(item['id'] == element_id for item in self.original_order):
                rospy.loginfo(f"ArUco ID {element_id} encontrado à esquerda.")
                break
            if self.ultrasound_left < 0.18:
                rospy.loginfo("Limite do ultrassom esquerdo alcançado.")
                break
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    # Busca para a direita
    def search_right(self, element_id):
        twist = Twist()
        twist.angular.z = -0.5  # Movimenta para a direita

        while not rospy.is_shutdown():
            if any(item['id'] == element_id for item in self.original_order):
                rospy.loginfo(f"ArUco ID {element_id} encontrado à direita.")
                break
            if self.ultrasound_right < 0.18:
                rospy.loginfo("Limite do ultrassom direito alcançado.")
                break
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def centralize_aruco(self, target_block_id):
        rospy.loginfo("Iniciando o processo de centralização do ArUco.")

        has_converged = False
        rate = rospy.Rate(10)  # Define a taxa de execução do loop (10 Hz)

        while not has_converged and not rospy.is_shutdown():
            # Processa os blocos detectados
            block_counter = 0
            for tag_id, position in self.current_detections:
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
                            rospy.loginfo("O robô está alinhado com o bloco.")
                            has_converged = True  # Marca o robô como alinhado
                            cmd_vel.linear.x = 0
                            cmd_vel.linear.y = 0
                
                # Publica os comandos de movimento
                self.cmd_vel_pub.publish(cmd_vel)
            
            # Aguarda até a próxima iteração
            rate.sleep()

    # Processo de pegar o bloco
    def pick_up_block(self):
        rospy.loginfo("Executando o processo de pegar o bloco.")
        self.pick_block_pub.publish("pegar bloco")
    
    def leave_block(self):
        rospy.loginfo("Executando o processo de largar o bloco.")
        self.pick_block_pub.publish("largar bloco")

    def wait_for_align_table_feedback(self):
        rospy.loginfo("Esperando feedback de alinhamento da mesa.")
        while not rospy.is_shutdown():
            feedback = rospy.wait_for_message("/align_table/feedback", Int32)
            if feedback.data == 1:
                rospy.loginfo("Feedback de alinhamento recebido.")
                break

    def wait_for_servo_feedback(self):
        rospy.loginfo("Esperando feedback do servo.")
        while not rospy.is_shutdown():
            feedback = rospy.wait_for_message("servo_control/move_to_pose/feedback", Int32)
            if feedback.data == 1:
                rospy.loginfo("Feedback do servo recebido.")
                break

    # Método principal de execução
    def execute(self):
        rospy.loginfo("Executando o processo.")

        # Inicia o alinhamento da mesa e espera feedback
        self.align_table_pub.publish("start")
        self.wait_for_align_table_feedback()  # Aguarda confirmação do alinhamento
        
        # Processo de mapeamento dos blocos
        self.mapping_process()

        if not self.mapping_done:
            rospy.logerr("Falha no mapeamento dos blocos.")
            self.is_executing = False  # Libera a execução em caso de falha
            return

        # Ordena o vetor de ArUcos pelo ID
        ordered_list = sorted(self.original_order, key=lambda x: x['id'])

        for i, aruco in enumerate(ordered_list):
            current_id = aruco['id']
            if i == 0:
                self.search_left(current_id)
            else:
                pos1 = i - 1 #Posição colocada no loop anterior (i-1)
                pos2 = next(index for index, item in enumerate(self.original_order) if item['id'] == current_id) #Posição do Bloco de ID (current_id) no vetor original
                rospy.info(f"A posições são: Origem{pos1} -> Destino{pos2} // Id Buscado = {current_id}")
                self.search_specific_id(pos1, pos2, current_id)
                
            self.centralize_aruco(current_id)
            rospy.sleep(0.3)
            self.pick_up_block()

            if i < len(ordered_list) - 1:
                if(i == 0):
                    next_id = self.original_order[i]['id']
                    pos1 = next(index for index, item in enumerate(self.original_order) if item['id'] == current_id) #Posição do Bloco de ID (current_id) no vetor original
                    pos2 = i #Posição onde o bloco deve ser colocado
                    rospy.info(f"A posições são: Origem{pos1} -> Destino{pos2} // Id Buscado = {next_id}")
                    self.search_specific_id(pos1, pos2, next_id)
                    self.centralize_aruco(next_id)

                    # Movimenta para frente usando o ultrassom
                    self.align_table_pub.publish("start")
                    self.wait_for_align_table_feedback()  # Aguarda confirmação do alinhamento

                    rospy.sleep(0.2)
                    # self.pub_move_time.publish("frente,1.1")
                    # rospy.sleep(1.4)
                else:
                    next_id = ordered_list[i]['id']
                    pos1 = next(index for index, item in enumerate(self.original_order) if item['id'] == current_id) #Posição do Bloco de ID (current_id) no vetor original
                    pos2 = i #Posição onde o bloco deve ser colocado
                    rospy.info(f"A posições são: Origem{pos1} -> Destino{pos2} // Id Buscado = {next_id}")

                     # Movimenta para frente usando o ultrassom
                    self.align_table_pub.publish("start")
                    self.wait_for_align_table_feedback()  # Aguarda confirmação do alinhamento

                    # self.pub_move_time.publish("frente,0.5")
                    # rospy.sleep(1.0)
                    self.search_specific_id(pos1, pos2, next_id)
                    
                    self.pub_move_time.publish("direita,1.1")
                    rospy.sleep(1.4)

                self.leave_block()
                rospy.sleep(0.2)
                
                self.servo_control_pub.publish("pose_inicial")
                self.wait_for_servo_feedback()  

                rospy.loginfo("Movimentando para trás até a posição original.")
                self.pub_move_time.publish("tras,1.1")
                rospy.sleep(1.4)

                self.align_table_pub.publish("start")
                self.wait_for_align_table_feedback()  # Aguarda confirmação do alinhamento

        rospy.loginfo("Processo concluído. Aguardando novo comando.")
        self.is_executing = False

