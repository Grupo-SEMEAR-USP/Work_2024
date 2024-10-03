#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
from robot_arm_control.srv import ControlArm, ControlArmResponse  

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
        return True, f"{parte} movido para a posição: {acao}"
    else:
        rospy.logwarn("Ação ou posição nomeada inválida para %s: %s", parte, acao)
        return False, f"Ação ou posição nomeada inválida para {parte}: {acao}"

# Função de callback para o serviço
def handle_control_arm(req):
    rospy.loginfo("Comando recebido: %s", req.command)

    # Divide a mensagem em duas partes: o grupo (parte) e a ação
    try:
        parte, acao = req.command.split(":")
        sucesso, mensagem = controlar_parte(parte, acao)
        return ControlArmResponse(success=sucesso, message=mensagem)
    except ValueError:
        rospy.logwarn("Formato da mensagem incorreto. Use 'parte:acao'.")
        return ControlArmResponse(success=False, message="Formato incorreto. Use 'parte:acao'.")

# Inicializa o nó ROS e o serviço
def control_arm_service():
    # Inicializa o moveit_commander e o ROS node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('control_arm_service_node')

    # Cria o serviço ROS que recebe comandos no formato 'parte:ação'
    service = rospy.Service('control_arm_service', ControlArm, handle_control_arm)

    rospy.loginfo("Serviço 'control_arm_service' disponível e aguardando comandos")

    # Mantém o script rodando até que o nó seja encerrado
    rospy.spin()

if __name__ == '__main__':
    try:
        control_arm_service()
    except rospy.ROSInterruptException:
        pass
