#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory

def trajectory_callback(data):
    rospy.loginfo("Trajetória recebida:")
    rospy.loginfo("Nomes das juntas: %s", data.joint_names)
    for idx, point in enumerate(data.points):
        rospy.loginfo("Ponto %d:", idx)
        rospy.loginfo("  Posições: %s", point.positions)
        rospy.loginfo("  Velocidades: %s", point.velocities)
        rospy.loginfo("  Acelerações: %s", point.accelerations)
        rospy.loginfo("  Esforços: %s", point.effort)
        rospy.loginfo("  Tempo desde o início: %s", point.time_from_start)

def listener():
    rospy.init_node('trajectory_listener', anonymous=True)
    rospy.Subscriber("/move_group/follow_joint_trajectory", JointTrajectory, trajectory_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
