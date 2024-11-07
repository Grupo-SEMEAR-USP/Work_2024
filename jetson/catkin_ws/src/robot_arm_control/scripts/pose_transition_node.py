import rospy
from robot_arm_control.msg import DualServoCommand, MoveToPoseCommand
from std_msgs.msg import Int32

feedback_received = None
last_command = None

def publish_pose_transition(pub_dual_servo, plat_angle, manip_angle):
    msg = DualServoCommand()
    msg.platform_angle = plat_angle  # Publica apenas o ângulo para o platform
    msg.manipulator_angle = manip_angle  # Publica apenas o ângulo para o manipulador
    rospy.loginfo(f"Publicando: platform_angle={plat_angle}, manipulator_angle={manip_angle}")
    pub_dual_servo.publish(msg)

def feedback_callback(msg):
    global feedback_received
    rospy.loginfo(f"Feedback recebido: {msg.data}")
    feedback_received = msg.data

def move_to_pose_callback(msg):
    global feedback_received, last_command

    pose_name = msg.pose_name
    rospy.loginfo(f"Recebido pedido para mover para {pose_name}")

    if last_command == pose_name:
        rospy.logwarn(f"Comando repetido detectado: {pose_name}. Publicando sucesso automático.")
        feedback_pub.publish(1)
        return
    else:
        last_command = pose_name

    try:
        pose = rospy.get_param(f"poses/{pose_name}")
        plat_angle = pose['plat_angle']
        manip_angle = pose['manip_angle']
        
        # Publica a transição de pose com os ângulos ajustados
        publish_pose_transition(pub_dual_servo, plat_angle, manip_angle)

        feedback_received = None
        timeout = rospy.Time.now() + rospy.Duration(10)
        while feedback_received is None and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        
        if feedback_received == 1:
            feedback_pub.publish(1)
            rospy.loginfo("Movimento bem-sucedido.")
        else:
            feedback_pub.publish(0)
            rospy.loginfo("Falha no movimento.")
            
    except KeyError:
        rospy.logerr(f"Pose {pose_name} não encontrada no servidor de parâmetros!")
        feedback_pub.publish(0)

def pose_transition_node():
    global pub_dual_servo, feedback_pub

    rospy.init_node('pose_transition_node', anonymous=True)

    pub_dual_servo = rospy.Publisher('/servo_control/dual_servo', DualServoCommand, queue_size=10)
    feedback_pub = rospy.Publisher('/servo_control/pose_feedback', Int32, queue_size=10)

    rospy.Subscriber('/servo_control/dual_servo/feedback', Int32, feedback_callback)
    rospy.Subscriber('/servo_control/move_to_pose', MoveToPoseCommand, move_to_pose_callback)

    rospy.loginfo("Pronto para receber comandos de movimentação.")
    rospy.spin()

if __name__ == '__main__':
    try:
        pose_transition_node()
    except rospy.ROSInterruptException:
        pass
