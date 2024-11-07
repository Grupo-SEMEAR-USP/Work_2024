#!/usr/bin/env python3

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image
from std_msgs.msg import String  # Tipo de mensagem para publicar o estado
import cv2
import numpy as np
import os
import json  # Para converter o dicionário em JSON para publicar

class ArucoColorDetection:
    def __init__(self):
        # Inicializa o nó do ROS
        rospy.init_node('aruco_color_detection', anonymous=True)

        # Configurações iniciais
        self.detected_blocks = {}
        self.camera_image = None
        self.image_received = False
        self.ocioso = False

        # Definição de limites para as cores verde e rosa
        self.green_lower_hsv = np.array([34, 87, 74])
        self.green_upper_hsv = np.array([71, 255, 129])
        self.pink_lower_hsv1 = np.array([0, 31, 87])
        self.pink_upper_hsv1 = np.array([32, 132, 152])
        self.pink_lower_hsv2 = np.array([135, 30, 62])
        self.pink_upper_hsv2 = np.array([179, 255, 255])

        # Cria o diretório de salvamento, se não existir
        self.image_save_dir = "/home/rmajetson/saved_images/"
        os.makedirs(self.image_save_dir, exist_ok=True)

        # Subscritores
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_detections_callback)
        rospy.Subscriber('/usb_cam/image_raw', Image, self.camera_callback)

        # Publisher para o estado dos blocos detectados
        self.blocks_pub = rospy.Publisher('/detected_blocks', String, queue_size=10)

    def create_mask(self, image, left_limit, right_limit):
        h, w = image.shape[:2]
        left_mask_points = np.array([(0, 0), (left_limit, 0), (left_limit, h), (0, h)])
        right_mask_points = np.array([(right_limit, 0), (w, 0), (w, h), (right_limit, h)])
        
        mask = np.ones_like(image) * 255
        cv2.fillPoly(mask, [left_mask_points], (0, 0, 0))
        cv2.fillPoly(mask, [right_mask_points], (0, 0, 0))
        
        masked_img = cv2.bitwise_and(image, mask)
        return masked_img

    def detect_block_color(self, roi, tag_id):
        hsv_image = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Máscaras para rosa e verde
        pink_mask1 = cv2.inRange(hsv_image, self.pink_lower_hsv1, self.pink_upper_hsv1)
        pink_mask2 = cv2.inRange(hsv_image, self.pink_lower_hsv2, self.pink_upper_hsv2)
        green_mask = cv2.inRange(hsv_image, self.green_lower_hsv, self.green_upper_hsv)

        # Contagem de pixels para cada máscara
        pink_pixels = cv2.countNonZero(pink_mask1) + cv2.countNonZero(pink_mask2)
        green_pixels = cv2.countNonZero(green_mask)

        # Salvando as imagens de threshold
        green_image_path = os.path.join(self.image_save_dir, f"green_mask_tag_{tag_id}.png")
        pink_image_path1 = os.path.join(self.image_save_dir, f"pink_mask1_tag_{tag_id}.png")
        pink_image_path2 = os.path.join(self.image_save_dir, f"pink_mask2_tag_{tag_id}.png")
        
        cv2.imwrite(green_image_path, green_mask)
        cv2.imwrite(pink_image_path1, pink_mask1)
        cv2.imwrite(pink_image_path2, pink_mask2)

        if pink_pixels > green_pixels:
            return "pink"
        elif green_pixels > pink_pixels:
            return "green"
        return "unknown"

    def camera_callback(self, img_msg):
        if not self.ocioso:
            try:
                if img_msg.encoding != "rgb8":
                    rospy.logwarn(f"Formato de imagem inesperado: {img_msg.encoding}")
                    return
                
            except Exception as e:
                rospy.logerr(f"Erro ao converter imagem: {e}")
                self.image_received = False

            print(self.detected_blocks)

            self.camera_image = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, 3)
            self.camera_image = cv2.cvtColor(self.camera_image, cv2.COLOR_RGB2BGR)
            self.image_received = True

    def tag_detections_callback(self, data):
        if len(data.detections) == 0 or not self.image_received or self.camera_image is None or self.ocioso:
            return

        # Aplicando máscara geral de ROI
        masked_image = self.create_mask(self.camera_image, 170, 470)
        for detection in data.detections:
            tag_id = detection.id[0]
            
            position = detection.pose.pose.pose.position
            x, y = int(position.x * 1000) + 220, int(position.y * 1000) + 190
            w, h = 180, 180

            # Ajustando ROI
            x = max(0, min(masked_image.shape[1] - w, x))
            y = max(0, min(masked_image.shape[0] - h, y))

            if 170 <= x <= 470 and abs(position.x * 1000) < 100:  # Verifica posição do centro
                roi = masked_image[y:y+h, x:x+w]
                color = self.detect_block_color(roi, tag_id)
                
                if color != "unknown":
                    self.detected_blocks[tag_id] = color
                    rospy.loginfo(f"Bloco ID {tag_id} identificado como {color}")

            if len([c for c in self.detected_blocks.values() if c == "pink"]) >= 2 and \
               len([c for c in self.detected_blocks.values() if c == "green"]) >= 2:
                rospy.sleep(1.0)
                rospy.loginfo("Detecção completa")
                self.ocioso = True
        
        # Publica o estado dos blocos detectados
        self.publish_detected_blocks()

    def publish_detected_blocks(self):
        # Converte o dicionário de blocos para JSON e publica
        detected_blocks_json = json.dumps(self.detected_blocks)
        self.blocks_pub.publish(detected_blocks_json)

    def run(self):
        rospy.loginfo("Ola1")
        
        # Loop para publicar a cada 0.5 segundos
        rate = rospy.Rate(10)  # 2 Hz
        while not rospy.is_shutdown():
            self.publish_detected_blocks()  # Publica o estado atual
            rate.sleep()
        
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = ArucoColorDetection()
        rospy.loginfo("Ola")
        detector.run()
    except rospy.ROSInterruptException:
        pass
