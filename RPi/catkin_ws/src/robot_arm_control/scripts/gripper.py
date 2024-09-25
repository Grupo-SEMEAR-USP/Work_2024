#!/usr/bin/env python

import rospy
from std_msgs.msg import String  # Tipo de mensagem que será recebida
import moveit_commander
import sys

# Função que controla o grupo especificado (gripper ou braço)
def controlar_parte(parte, acao):
    # Inicializa o MoveGroup para o grupo fornecido (gripper ou arm)
    move_group = moveit_commander.MoveGroupCommander(parte)

    # Verifica se a ação solicitada é uma posição nomeada válida
    if acao in move_group.get_named_targets():
        # Define a posição alvo como a ação recebida
        move_group.set_named_target(acao)
        # Executa o movimento
        move_group.go(wait=True)
        rospy.loginfo("%s movido para a posição: %s", parte, acao)
    else:
        rospy.logwarn("Ação ou posição nomeada inválida para %s: %s", parte, acao)

# Função de callback para processar as mensagens
def callback_function(data):
    rospy.loginfo("Mensagem recebida: %s", data.data)

    # Divide a mensagem em duas partes: o grupo (parte) e a ação
    try:
        parte, acao = data.data.split(":")
        controlar_parte(parte, acao)
    except ValueError:
        rospy.logwarn("Formato da mensagem incorreto. Use 'parte:acao'.")

# Inicializa o nó ROS e o subscriber
def listener():
    # Inicializa o moveit_commander e o ROS node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('controle_geral_node', anonymous=True)

    # Subscrição ao tópico 'controle_geral'. O tipo de mensagem é std_msgs/String
    rospy.Subscriber("controle_geral", String, callback_function)

    # Mantém o script rodando até que o nó seja encerrado
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
