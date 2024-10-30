#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Range
from apriltag_ros.msg import AprilTagDetectionArray

class ZigZagSearch:
    def __init__(self):
        rospy.init_node('search_block')

        self.aruco_reference_id = None
        self.distance_threshold = 0.15  # 15 cm

        self.pub_move_time = rospy.Publisher('/move_time', String, queue_size=10)
        self.pub_feedback = rospy.Publisher('/search_block/feedback', Int32, queue_size=10)
        self.sub_search = rospy.Subscriber('/search_block', String, self.search_callback)
        self.sub_tag_detections = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_detection_callback)
        self.sub_ultrasound_right = rospy.Subscriber('/ultrasound_right', Range, self.ultrasound_callback)

        self.rate = rospy.Rate(50)
        self.movement_active = False
        self.tag_detected_count = 0
        self.current_distance = float('inf')  # Inicializa com um valor alto

    def search_callback(self, msg):
        try:
            command, block_id = msg.data.split(',')
            block_id = int(block_id.strip())

            if command == 'start':
                self.aruco_reference_id = block_id
                self.movement_active = True
                self.tag_detected_count = 0
                rospy.loginfo(f"Movimento zigue-zague iniciado para o bloco {self.aruco_reference_id}.")
                self.perform_zigzag()
            else:
                rospy.logwarn(f"Comando desconhecido: {command}")
        
        except ValueError as e:
            rospy.logerr(f"Erro ao interpretar a mensagem: {msg.data}. Formato esperado: 'start, <ID_do_bloco>'")

    def ultrasound_callback(self, data):
        self.current_distance = data.range / 100  # Atualiza a distância com a leitura do sensor ultrassônico

    def perform_zigzag(self):
        if self.movement_active:
            try:
                while self.current_distance > self.distance_threshold and self.movement_active:
                    rospy.loginfo("Executando movimento de zigue-zague...")

                    rospy.loginfo("Movendo para a direita por 2.0 segundos.")
                    self.pub_move_time.publish("direita,2.0")
                    rospy.sleep(2.3)
                    if self.tag_detected_count >= 10:
                        rospy.loginfo("ArUco de referência detectado 10 vezes consecutivas. Parando o movimento.")
                        self.stop_movement()
                        return

                    rospy.loginfo("Movendo para trás por 1.1 segundos.")
                    self.pub_move_time.publish("tras,1.1")
                    rospy.sleep(1.4)
                    if self.tag_detected_count >= 10:
                        rospy.loginfo("ArUco de referência detectado 10 vezes consecutivas. Parando o movimento.")
                        self.stop_movement()
                        return
                    if self.current_distance <= self.distance_threshold:
                        rospy.loginfo(f"Distância de {self.current_distance:.2f} metros atingida. Parando o movimento de zigue-zague.")

                        self.pub_move_time.publish("frente,1.1")
                        rospy.sleep(1.4)
                        self.stop_movement()
                        self.perform_reverse_zigzag()
                        return

                    rospy.loginfo("Movendo para a direita por 2.0 segundos.")
                    self.pub_move_time.publish("direita,2.0")
                    rospy.sleep(2.3)
                    if self.tag_detected_count >= 10:
                        rospy.loginfo("ArUco de referência detectado 10 vezes consecutivas. Parando o movimento.")
                        self.stop_movement()
                        return

                    rospy.loginfo("Movendo para a frente por 1.1 segundos.")
                    self.pub_move_time.publish("frente,1.1")
                    rospy.sleep(1.4)
                    if self.tag_detected_count >= 10:
                        rospy.loginfo("ArUco de referência detectado 10 vezes consecutivas. Parando o movimento.")
                        self.stop_movement()
                        return

                    # Verifica a distância para interromper o movimento ao atingir a distância limite
                    if self.current_distance <= self.distance_threshold:
                        rospy.loginfo(f"Distância de {self.current_distance:.2f} metros atingida. Parando o movimento de zigue-zague.")
                        self.stop_movement()
                        self.perform_reverse_zigzag()
                        return

            except Exception as e:
                rospy.logerr("Erro durante o movimento: %s", str(e))
                self.pub_feedback.publish(0)
                self.movement_active = False
                return

    def perform_reverse_zigzag(self):
        try:
            rospy.loginfo("Executando movimento de zigue-zague reverso...")

            rospy.sleep(1)

            if self.tag_detected_count >= 10:
                rospy.loginfo("ArUco de referência detectado 10 vezes consecutivas. Parando o movimento.")
                self.stop_movement()
                return

            rospy.loginfo("Movendo para a esquerda por 2.0 segundos.")
            self.pub_move_time.publish("esquerda,2.0")
            rospy.sleep(2.3)

            for cycle in range(3):
                rospy.loginfo(f"Executando ciclo inverso {cycle + 1}/3")

                if self.tag_detected_count >= 10:
                    rospy.loginfo("ArUco de referência detectado 10 vezes consecutivas. Parando o movimento.")
                    self.stop_movement()
                    return

                rospy.loginfo("Movendo para trás por 1.1 segundos.")
                self.pub_move_time.publish("tras,1.1")
                rospy.sleep(1.4)

                if self.tag_detected_count >= 10:
                    rospy.loginfo("ArUco de referência detectado 10 vezes consecutivas. Parando o movimento.")
                    self.stop_movement()
                    return

                rospy.loginfo("Movendo para a esquerda por 2.0 segundos.")
                self.pub_move_time.publish("esquerda,2.0")
                rospy.sleep(2.3)

                if self.tag_detected_count >= 10:
                    rospy.loginfo("ArUco de referência detectado 10 vezes consecutivas. Parando o movimento.")
                    self.stop_movement()
                    return

                rospy.loginfo("Movendo para a frente por 1.1 segundos.")
                self.pub_move_time.publish("frente,1.1")
                rospy.sleep(1.4)

                if self.tag_detected_count >= 10:
                    rospy.loginfo("ArUco de referência detectado 10 vezes consecutivas. Parando o movimento.")
                    self.stop_movement()
                    return
                
                if cycle < 2:
                    rospy.loginfo("Movendo para a esquerda por 2.0 segundos.")
                    self.pub_move_time.publish("esquerda,2.0")
                    rospy.sleep(2.3)

            rospy.loginfo("Movimento de zigue-zague reverso completo.")
            self.pub_feedback.publish(1)

        except Exception as e:
            rospy.logerr("Erro durante o movimento inverso: %s", str(e))
            self.pub_feedback.publish(0)
            self.movement_active = False

    def tag_detection_callback(self, data):
        for detection in data.detections:
            tag_id = detection.id[0]
            if tag_id == self.aruco_reference_id:
                rospy.loginfo(f"ArUco de referência {self.aruco_reference_id} detectado.")
                self.tag_detected_count += 1
                rospy.loginfo(f"Detecções consecutivas: {self.tag_detected_count}/10")
                if self.tag_detected_count >= 10:
                    self.stop_movement()
                break

    def stop_movement(self):
        if self.movement_active:
            rospy.loginfo("Parando o movimento.")
            self.pub_move_time.publish("parar,0")
            self.pub_feedback.publish(1)
            self.movement_active = False
            self.tag_detected_count = 0

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        zigzag = ZigZagSearch()
        zigzag.run()
    except rospy.ROSInterruptException:
        pass
