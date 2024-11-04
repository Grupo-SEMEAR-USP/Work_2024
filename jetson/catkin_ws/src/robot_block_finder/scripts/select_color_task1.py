#!/usr/bin/env python3

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image
import cv2
import numpy as np

pink_blocks = []
green_blocks = []
camera_image = None
image_received = False  # Flag para indicar quando a imagem foi recebida

def detect_block_color(image, x, y, w, h, tag_id):


    rospy.loginfo(f"Detectando cor para o bloco ID {tag_id} na região x:{x}, y:{y}, w:{w}, h:{h}")
    roi = image[y:y+h, x:x+w]

    avg_color_per_row = np.mean(roi, axis=0)
    avg_color = np.mean(avg_color_per_row, axis=0)
    avg_color_rgb = (int(avg_color[2]), int(avg_color[1]), int(avg_color[0]))  # Convertendo para RGB
    rospy.loginfo(f"Valor médio RGB para o bloco ID {tag_id}: {avg_color_rgb}")
    
    hsv_image = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # Limites refinados para as cores rosa e verde em HSV
    pink_lower = (150, 100, 100)
    pink_upper = (170, 255, 255)
    green_lower = (40, 70, 70)
    green_upper = (80, 255, 255)

    # Máscaras para rosa e verde
    pink_mask = cv2.inRange(hsv_image, pink_lower, pink_upper)
    green_mask = cv2.inRange(hsv_image, green_lower, green_upper)

    # Contagem de pixels para cada máscara
    pink_pixels = cv2.countNonZero(pink_mask)
    green_pixels = cv2.countNonZero(green_mask)

    # Escolhe a máscara e a cor
    if pink_pixels > green_pixels:
        selected_mask = pink_mask
        color_name = "pink"
    elif green_pixels > pink_pixels:
        selected_mask = green_mask
        color_name = "green"
    else:
        rospy.loginfo(f"Bloco ID {tag_id} identificado como cor desconhecida")
        return "unknown"

    # Aplica a máscara na ROI para salvar apenas os pixels identificados
    result_image = cv2.bitwise_and(roi, roi, mask=selected_mask)
    filename = f"Work_2024/jetson/catkin_ws/src/robot_utils/imgs/detected_{color_name}_block_id_{tag_id}.png"
    cv2.imwrite(filename, result_image)
    # rospy.loginfo(f"Imagem salva: {filename}")

    filename = f"Work_2024/jetson/catkin_ws/src/robot_utils/imgs/roi_{color_name}_block_id_{tag_id}.png"
    cv2.imwrite(filename, roi)
    # rospy.loginfo(f"ROI salva: {filename}")

    return color_name

def camera_callback(img_msg):
    global camera_image, image_received
    try:
        if img_msg.encoding != "rgb8":
            rospy.logwarn(f"Formato de imagem inesperado: {img_msg.encoding}")
            return

        # Converte a imagem diretamente para um array numpy
        camera_image = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, 3)
        
        # Converte de RGB para BGR para compatibilidade com OpenCV
        camera_image = cv2.cvtColor(camera_image, cv2.COLOR_RGB2BGR)
        
        image_received = True
    except Exception as e:
        rospy.logerr(f"Erro ao converter imagem: {e}")
        image_received = False

def tag_detections_callback(data):
    global pink_blocks, green_blocks, camera_image, image_received

    if len(data.detections) == 0:
        return
    elif not image_received or camera_image is None:
        rospy.loginfo("Imagem da câmera ainda não disponível ou está None")
        return

    for detection in data.detections:
        tag_id = detection.id[0]
        position = detection.pose.pose.pose.position

        x = int(position.x * 1000) + 300
        y = int(position.y * 1000) + 220
        w, h = 100, 100

        if camera_image is not None:
            x = max(0, min(camera_image.shape[1] - w, x))
            y = max(0, min(camera_image.shape[0] - h, y))

        color = detect_block_color(camera_image, x, y, w, h, tag_id)

        if color == "pink" and tag_id not in pink_blocks:
            pink_blocks.append(tag_id)
        elif color == "green" and tag_id not in green_blocks:
            green_blocks.append(tag_id)

        rospy.loginfo(f"Detecção completa: Rosas: {pink_blocks}, Verdes: {green_blocks}")

        # Verifica se o objetivo foi atingido
        if len(pink_blocks) >= 2 and len(green_blocks) >= 2:   
            rospy.loginfo(f"Detecção completa: Rosas: {pink_blocks}, Verdes: {green_blocks}")
            rospy.signal_shutdown("Detecção completa")

def main():
    rospy.init_node('aruco_color_detection', anonymous=True)
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_detections_callback)
    rospy.Subscriber('/usb_cam/image_raw', Image, camera_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
