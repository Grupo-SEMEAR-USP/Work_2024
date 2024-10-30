#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32

def move_callback(msg):
    try:
        direction, duration = msg.data.split(',')
        duration = float(duration)

        move_cmd = Twist()

        if direction == 'frente':
            move_cmd.linear.x = 0.2
        elif direction == "parar":
            move_cmd.linear.x = 0
            move_cmd.linear.z = 0
        elif direction == 'tras':
            move_cmd.linear.x = -0.2
        elif direction == 'esquerda':
            move_cmd.angular.z = 0.6
        elif direction == 'direita':
            move_cmd.angular.z = -0.6
        else:
            rospy.logwarn("Direção não reconhecida. Use 'frente', 'tras', 'esquerda' ou 'direita'.")
            return

        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        feedback_pub = rospy.Publisher('/move_time/feedback', Int32, queue_size=10)
        rate = rospy.Rate(50)  # 50 Hz
        
        start_time = rospy.Time.now()
        duration_time = rospy.Duration(duration)

        while rospy.Time.now() - start_time < duration_time:
            pub.publish(move_cmd)
            rate.sleep()
        
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)

        rospy.loginfo(f"Movimento completado: {direction} por {duration} segundos.")
        
        feedback_pub.publish(1)

    except Exception as e:
        rospy.logerr(f"Erro ao processar a mensagem: {str(e)}")

def listener():
    rospy.init_node('move_time_listener', anonymous=True)

    rospy.Subscriber("move_time", String, move_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
