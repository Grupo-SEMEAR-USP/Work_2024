#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

# Importar as funções de cada segmento
from line_follower_paths import seg_12, seg_13, seg_14, seg_ALL5, seg_ALL6

def goal_callback(msg):
    """
    Callback executado quando uma nova mensagem é recebida no tópico "goal_line_follower".
    Dependendo do valor da mensagem, a função correspondente é chamada.
    """
    goal_value = msg.data
    rospy.loginfo(f"Recebido novo goal: {goal_value}")

    if goal_value == 12:
        rospy.loginfo("Executando o segmento 12.")
        seg_12()
    elif goal_value == 13:
        rospy.loginfo("Executando o segmento 13.")
        seg_13()
    elif goal_value == 14:
        rospy.loginfo("Executando o segmento 14.")
        seg_14()
    elif goal_value == 15:
        rospy.loginfo("Executando o segmento 15 (seg_ALL5).")
        seg_ALL5()
    elif goal_value == 16:
        rospy.loginfo("Executando o segmento 16 (seg_ALL6).")
        seg_ALL6()
    else:
        rospy.loginfo("Goal desconhecido. Nenhuma ação executada.")

def listener():
    """
    Função principal que inicializa o nó e se inscreve no tópico "goal_line_follower".
    """
    rospy.init_node('goal_line_follower_listener', anonymous=True)
    
    # Inscreve-se no tópico "goal_line_follower", esperando por valores inteiros
    rospy.Subscriber('goal_line_follower', Int32, goal_callback)

    # Mantém o nó em execução até que seja encerrado
    rospy.loginfo("Aguardando novos goals no tópico 'goal_line_follower'...")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node encerrado.")
