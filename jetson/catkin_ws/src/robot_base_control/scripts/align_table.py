#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Range

class MoveToTarget:
    def __init__(self):
        rospy.init_node('move_to_target', anonymous=True)
        self.ultrasound_sub = rospy.Subscriber("/ultrasound_front", Range, self.ultrasound_callback)
        self.start_sub = rospy.Subscriber("/align_table", String, self.start_callback)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.feedback_pub = rospy.Publisher("/align_table/feedback", Int32, queue_size=10)
        self.twist = Twist()
        self.start_movement = False
        self.recent_distances = []

    def start_callback(self, msg):
        if msg.data == "start":
            self.start_movement = True

    def ultrasound_callback(self, data):
        if not self.start_movement:
            return

        center_distance = data.range / 100  # Acessa o valor da distância no campo `range`
        rospy.loginfo("Distância do sensor ultrassônico: %.2f metros" % center_distance)

        self.recent_distances.append(center_distance)
        if len(self.recent_distances) > 3:
            self.recent_distances.pop(0)

        if center_distance < 0.09:
            rospy.loginfo("Distância menor que 0.10 metros. Movendo para trás...")
            self.twist.linear.x = -0.2
        elif center_distance > 0.18:
            rospy.loginfo("Distância maior que 0.10 metros. Movendo para frente...")
            self.twist.linear.x = 0.2
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            rospy.loginfo("Parando, 3 leituras consecutivas abaixo de 0.25 metros!")
            self.feedback_pub.publish(1)
            rospy.loginfo("Feedback publicado: 1")
            self.start_movement = False
            
            self.vel_pub.publish(self.twist)
            return 

        self.vel_pub.publish(self.twist)

if __name__ == '__main__':
    try:
        move_to_target = MoveToTarget()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
