#!/usr/bin/env python3

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist

has_converged = False

def tag_detections_callback(data):
    global has_converged
    
    if len(data.detections) == 0:
        rospy.loginfo("No tags detected.")
        return
    
    if has_converged:
        rospy.loginfo("Already converged, not adjusting.")
        return

    for detection in data.detections:
        position = detection.pose.pose.pose.position
        tag_id = detection.id[0]

        rospy.loginfo("Tag detected - ID: %d", tag_id)
        rospy.loginfo("Position [x: %f, y: %f, z: %f]", position.x, position.y, position.z)
        
        cmd_vel = Twist()

        if abs(position.y) > 0.05: 
            cmd_vel.linear.x = 0.15 if position.x > 0 else -0.15
            cmd_vel.linear.y = 0.0 
        else:
            cmd_vel.linear.x = 0.0  
            if abs(position.x) > 0.01:  
                cmd_vel.linear.y = 0.15 if position.y > 0 else -0.15
            else:
                cmd_vel.linear.y = 0.0  
                vel_pub.publish(cmd_vel)
                has_converged = True
                rospy.loginfo("Robot has converged.")

        if has_converged == False:
            vel_pub.publish(cmd_vel)

def main():
    global vel_pub

    rospy.init_node('align_block', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_detections_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
