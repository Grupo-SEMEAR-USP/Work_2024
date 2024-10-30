#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

# Linear forward speed
LINEAR_SPEED = 0.4
# Proportional constant base value for angular speed when turning
KP_BASE = 1.5 / 100
# Minimum area for tracking contours
MIN_AREA_TRACK = 1100
# Maximum error value before triggering a search mode
MAX_ERROR = 300

#1 para ida (Curvas à esquerda)
#0 para a volta (Curvas à direita)
SIDE_TO_GO = 1

class ImageProcessor:
    def __init__(self):
        rospy.init_node('line_follower', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/camera/image_raw", Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.mask_pub = rospy.Publisher("/mask_image", Image, queue_size=1)  # Publisher for the mask image
        self.previous_errors = []
        self.last_known_line_position = None
        self.search_mode = False

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.process_image(cv_image)
        except CvBridgeError as e:
            print(e)

    def process_image(self, current_frame):
        # Create a masked image where only below 300 pixels in y-axis are visible
        masked_frame = current_frame

        hsv_image = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2HSV)

        lower_blue = np.array([0, 0, 38])  # Adjust these values to match the color of the line.
        upper_blue = np.array([21, 47, 60])
        blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

        # Show the binary mask image (black and white)
        cv2.imshow("Mask", blue_mask)
        cv2.waitKey(1)

        # Publish the black and white mask image
        try:
            mask_image_msg = self.bridge.cv2_to_imgmsg(blue_mask, encoding="mono8")
            self.mask_pub.publish(mask_image_msg)
        except CvBridgeError as e:
            rospy.logerr("Failed to convert and publish mask image: %s", e)

        line = self.get_contour_data(blue_mask)

        cmd_vel = Twist()

        if line:
            # Line found, exit search mode
            self.search_mode = False
            self.last_known_line_position = line  # Save last known line position

            x = line['x']
            y = line['y']
            _, width = masked_frame.shape[:2]
            error = x - width // 2

            # Smoothed error using average
            error = self.smooth_error(error)

            # Dynamically adjust KP based on the error magnitude
            KP = self.adjust_kp(error)

            # Compute control commands
            cmd_vel.linear.x = LINEAR_SPEED
            cmd_vel.angular.z = float(error) * -KP

            # Print commands in the terminal
            print("Linear Speed:", cmd_vel.linear.x)
            print("Angular Velocity:", cmd_vel.angular.z)

            # Draw centroid on the masked frame
            cv2.circle(masked_frame, (x, y), 5, (0, 255, 0), -1)

        else:
            # Line not found, enter search mode
            self.search_mode = True
            if SIDE_TO_GO:
                print("Linha não detectada! Parando e girando para a esquerda.")
                cmd_vel.linear.x = 0.0  # Stop forward motion
                cmd_vel.angular.z = 2.0  # Rotate to the left
            else:
                print("Linha não detectada! Parando e girando para a direita.")
                cmd_vel.linear.x = 0.0  # Stop forward motion
                cmd_vel.angular.z = -2.0  # Rotate to the left


        # Show the masked frame with the centroid
        cv2.imshow("Masked Processed Image", masked_frame)
        cv2.waitKey(1)

        # Publish velocity command
        self.cmd_vel_pub.publish(cmd_vel)

    def get_contour_data(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        line = {}
        max_area = 0  # Variable to store the largest contour area

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > MIN_AREA_TRACK and area > max_area:
                M = cv2.moments(contour)
                if M['m00'] > 0:
                    max_area = area
                    line['x'] = int(M["m10"] / M["m00"])
                    line['y'] = int(M["m01"] / M["m00"])

        return line if line else None

    def smooth_error(self, current_error):
        """
        Suaviza o erro usando uma média móvel para reduzir oscilações bruscas.
        """
        if len(self.previous_errors) > 10:
            self.previous_errors.pop(0)
        self.previous_errors.append(current_error)
        return sum(self.previous_errors) / len(self.previous_errors)

    def adjust_kp(self, error):
        """
        Ajusta o valor de KP dinamicamente com base no valor absoluto do erro.
        """
        dynamic_kp = KP_BASE * (1 + abs(error) / 100)
        return dynamic_kp

def create_mask(image):
    """
    Função que cria uma máscara para a parte inferior da imagem.
    """
    h, w = image.shape[:2]  # h = altura, w = largura

    top_rec_x, top_rec_y = 0, 220
    top_rec_x2, top_rec_y2 = w, 220

    # Defina os pontos para a máscara
    tmask_points = np.array([(top_rec_x, top_rec_y), (top_rec_x2, top_rec_y2), (w, h), (0, h)])

    # Crie a máscara do mesmo tamanho da imagem, mas toda preta (zeros)
    mask = np.zeros_like(image)

    # Preencha a máscara com branco (255, 255, 255) na região definida por tmask_points
    cv2.fillPoly(mask, [tmask_points], (255, 255, 255))

    # Aplique a máscara à imagem original usando a operação bitwise_and
    masked_img = cv2.bitwise_and(image, mask)

    return masked_img

def main():
    try:
        ImageProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
