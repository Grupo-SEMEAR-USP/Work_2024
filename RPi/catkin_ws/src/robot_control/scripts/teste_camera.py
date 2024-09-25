import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

bridge = CvBridge()

def depth_callback(data):
    try:
        # Converter a imagem de profundidade ROS para uma imagem OpenCV
        depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        # Pegar a distância do objeto mais próximo (por exemplo, o centro da imagem)
        
        height, width = depth_image.shape
        center_distance = depth_image[height // 2, width // 2]
        rospy.loginfo(f"Distância ao objeto no centro da imagem: {center_distance} metros")
    except Exception as e:
        rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('depth_camera_listener')
    depth_sub = rospy.Subscriber('/depth_camera/depth/image_raw', Image, depth_callback)
    rospy.spin()
