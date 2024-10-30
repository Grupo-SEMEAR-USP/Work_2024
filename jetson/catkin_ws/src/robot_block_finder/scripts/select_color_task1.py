#!/usr/bin/env python3

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

bridge = CvBridge()
red_blocks = []
blue_blocks = []
camera_image = None

def detect_block_color(image, x, y, w, h, tag_id):
    roi = image[y:y+h, x:x+w]
    hsv_image = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    red_lower = (0, 100, 100)
    red_upper = (10, 255, 255)
    blue_lower = (100, 150, 0)
    blue_upper = (140, 255, 255)

    red_mask = cv2.inRange(hsv_image, red_lower, red_upper)
    blue_mask = cv2.inRange(hsv_image, blue_lower, blue_upper)

    red_pixels = cv2.countNonZero(red_mask)
    blue_pixels = cv2.countNonZero(blue_mask)

    if red_pixels > blue_pixels:
        return "red"
    elif blue_pixels > red_pixels:
        return "blue"
    else:
        return "unknown"

def camera_callback(img_msg):
    global camera_image
    try:
        camera_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except cv2.CvBridgeError as e:
        rospy.logerr(f"Erro ao converter imagem: {e}")

def tag_detections_callback(data):
    global red_blocks, blue_blocks, camera_image

    if len(data.detections) == 0 or camera_image is None:
        return

    for detection in data.detections:
        tag_id = detection.id[0]
        position = detection.pose.pose.pose.position

        x = int(position.x * 1000) + 300
        y = int(position.y * 1000) + 220
        w, h = 100, 100
        x = max(0, min(camera_image.shape[1] - w, x))
        y = max(0, min(camera_image.shape[0] - h, y))

        color = detect_block_color(camera_image, x, y, w, h, tag_id)

        if color == "red" and tag_id not in red_blocks:
            red_blocks.append(tag_id)
            rospy.loginfo(f"Bloco vermelho adicionado: ID {tag_id}")
        elif color == "blue" and tag_id not in blue_blocks:
            blue_blocks.append(tag_id)
            rospy.loginfo(f"Bloco azul adicionado: ID {tag_id}")

        # Check if the goal has been reached
        if len(red_blocks) >= 2 and len(blue_blocks) >= 2:   
            rospy.loginfo(f"Vermelhos: {red_blocks}")
            rospy.loginfo(f"Azuis: {blue_blocks}")
            rospy.signal_shutdown("Detecção completa")

def main():
    rospy.init_node('aruco_color_detection', anonymous=True)
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_detections_callback)
    rospy.Subscriber('/camera/camera/image_raw', Image, camera_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
