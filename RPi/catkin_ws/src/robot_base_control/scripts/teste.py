#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def alignment_callback(data):
    rospy.loginfo("Alignment signal received: %s", data.data)
    # Ação a ser realizada quando o sinal for recebido
    if data.data == "aligned":
        rospy.loginfo("Executing action on alignment.")

def main():
    rospy.init_node('alignment_listener', anonymous=True)
    rospy.Subscriber('/alignment_signal', String, alignment_callback)
    rospy.loginfo("Waiting for alignment signal...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
