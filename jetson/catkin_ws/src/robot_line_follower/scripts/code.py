#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from line_follower import LineFollower
from movement_controller import MovementController
from std_msgs.msg import Int32
import rospy

''' (Defining Variables For Line Follower)

turn_side = 1 - Virar à esquerda
turn_side = 2 - Virar à direita

turn_at_intersection = N° da intersecção para virar (1, 2, 3, 4)

lost_line_turn = 0 - Andar reto até achar a linha
lost_line_turn = 1 - Virar à esquerda se perder a linha
lost_line_turn = 2 - Virar à direita se perder a linha

side_surpass = 1 - Fazer a ultrapassagem pela esquerda se encontrar obstáculo
side_surpass = 2 - Fazer a ultrapassagem pela direita se encontrar obstáculo
'''

def seg_12():

    #Giro inicial de 90°
    movement_sequence = [
        {'action': 'turn', 'direction': 'left_z', 'duration': 5.65},
        {'action': 'turn', 'direction': 'left', 'duration': 5.65},
    ]
    #Curva
    move = MovementController() 
    move.start()
    #move.use_camera = False
    move.execute_movement_sequence(movement_sequence)

    while not rospy.is_shutdown():
        if move.sequence_complete == 1:  
            rospy.loginfo("Movimento concluído. Publicando feedback e encerrando.")
            move.stop_controller()  # Parar controlador e timers
            move.stop_feedback()
            break

    movement_sequence = [
        {'action': 'move_forward', 'duration': 2.1},
        {'action': 'turn', 'direction': 'left', 'duration': 2.2},
    ]
    
    turn_side = 1  # Definir como virar à esquerda
    turn_at_intersection = 1  # Virar na 2ª interseção
    lost_line_turn = 0  # Virar à esquerda se perder a linha
    surpass_side = 1  # Fazer a ultrapassagem pela esquerda

    #Seguidor de linha, parte 1
    follower = LineFollower(turn_side, turn_at_intersection, lost_line_turn, surpass_side)

    rospy.loginfo("Iniciando o seguidor de linha...")
    follower.start()

    rospy.loginfo("Seguidor de linha finalizado. Executando a curva para a esquerda.")

    #Curva
    move = MovementController()
    move.start()
    move.use_camera = False
    move.execute_movement_sequence(movement_sequence)

    while not rospy.is_shutdown():
        if move.sequence_complete == 1: 
            move.stop_controller()  # Parar controlador e timers 
            rospy.loginfo("Movimento concluído. Publicando feedback e encerrando.")
            move.set_feedback_scheduler(1.0)  # Publica o valor 1 no feedback
            rospy.sleep(0.1)
            move.stop_feedback()
            break

    #Processo de andar at'proximidade X

def seg_13():

    # #Giro inicial de 90°
    # movement_sequence = [
    #     {'action': 'turn', 'direction': 'left', 'duration': 2.65},
    #     {'action': 'move_forward', 'duration': 2.5},
    # ]
    # #Curva
    # move = MovementController()
    # move.start()
    # move.use_camera = False
    # move.execute_movement_sequence(movement_sequence)

    # while not rospy.is_shutdown():
    #     if move.sequence_complete == 1:  
    #         rospy.loginfo("Movimento concluído. Publicando feedback e encerrando.")
    #         move.stop_controller()  # Parar controlador e timers
    #         move.stop_feedback()
    #         break

    movement_sequence = [
        {'action': 'move_forward', 'duration': 2.2},
        {'action': 'turn', 'direction': 'left', 'duration': 2.2},
    ]
    
    turn_side = 1  # Definir como virar à esquerda
    turn_at_intersection = 2 # Virar na 2ª interseção
    lost_line_turn = 0  # Virar à esquerda se perder a linha
    surpass_side = 1  # Fazer a ultrapassagem pela esquerda

    follower = LineFollower(turn_side, turn_at_intersection, lost_line_turn, surpass_side)

    rospy.loginfo("Iniciando o seguidor de linha...")
    follower.start()

    rospy.loginfo("Seguidor de linha finalizado. Executando a curva para a esquerda.")

    #Curva
    move = MovementController()
    move.start()
    #move.use_camera = False
    move.execute_movement_sequence(movement_sequence)

    while not rospy.is_shutdown():
        if move.sequence_complete == 1:  
            move.stop_controller()  # Parar controlador e timers
            rospy.loginfo("Movimento concluído. Publicando feedback e encerrando.")
            move.set_feedback_scheduler(0.0)  # Publica o valor 1 no feedback
            move.stop_feedback()
            break
    #Seguidor de linha, parte 2

    rospy.sleep(2.0)

    movement_sequence = [
        {'action': 'move_forward', 'duration': 2.15},
        {'action': 'turn', 'direction': 'right', 'duration': 2.0},
    ]

    turn_side = 2  # Definir como virar à esquerda
    turn_at_intersection = 1  # Virar na 2ª interseção
    lost_line_turn = 0  # Virar à esquerda se perder a linha
    surpass_side = 1  # Fazer a ultrapassagem pela esquerda

    follower2 = LineFollower(turn_side, turn_at_intersection, lost_line_turn, surpass_side)

    rospy.loginfo("Iniciando o seguidor de linha...")
    follower2.start()
    rospy.loginfo("Seguidor de linha finalizado. Executando a curva para a direita...")

    #Curva
    move1 = MovementController()
    move1.start()
    move1.use_camera = False
    move1.execute_movement_sequence(movement_sequence)

     # Loop simples, sem rospy.Rate, para monitorar a variável de controle
    while not rospy.is_shutdown():
        if move1.sequence_complete == 1:  # Verifica se o movimento foi concluído
            move1.stop_controller()  # Parar controlador e timers
            rospy.loginfo("Movimento concluído. Publicando feedback e encerrando.")
            move1.set_feedback_scheduler(1.0)  # Publica o valor 1 no feedback
            rospy.sleep(0.1)
            move1.stop_feedback()
            break  

    # Encerrando o ROS após o fim do movimento

def seg_14():

    #Giro inicial de 90°
    movement_sequence = [
        {'action': 'turn', 'direction': 'left', 'duration': 2.6},
    ]
    #Curva
    move = MovementController()
    move.start()
    move.use_camera = False
    move.execute_movement_sequence(movement_sequence)

    while not rospy.is_shutdown():
        if move.sequence_complete == 1:  
            rospy.loginfo("Movimento concluído. Publicando feedback e encerrando.")
            move.stop_controller()  # Parar controlador e timers
            move.stop_feedback()
            break

    movement_sequence = [
        {'action': 'move_forward', 'duration': 0.8},
        {'action': 'turn', 'direction': 'right', 'duration': 2.4},
    ]
    
    turn_side = 1  # Definir como virar à esquerda
    turn_at_intersection = 2  # Virar na 2ª interseção
    lost_line_turn = 2  # Virar à direita se perder a linha
    surpass_side = 1  # Fazer a ultrapassagem pela esquerda

    follower = LineFollower(turn_side, turn_at_intersection, lost_line_turn, surpass_side)

    rospy.loginfo("Iniciando o seguidor de linha...")
    follower.start()

    rospy.loginfo("Seguidor de linha finalizado. Executando a curva para a esquerda.")

    #Curva
    move = MovementController()
    move.start()
    move.use_camera = False
    move.execute_movement_sequence(movement_sequence)

    while not rospy.is_shutdown():
        if move.sequence_complete == 1:  
            move.stop_controller()
            rospy.loginfo("Movimento concluído. Publicando feedback e encerrando.")
            move.stop_feedback()
            break

    #Seguidor de linha, parte 2

    turn_side = 1  # Definir como virar à esquerda
    turn_at_intersection = 1  # Virar na 2ª interseção
    lost_line_turn = 0  # Andar rto até ahar a linha
    surpass_side = 1  # Fazer a ultrapassagem pela esquerda

    follower2 = LineFollower(turn_side, turn_at_intersection, lost_line_turn, surpass_side)

    rospy.loginfo("Iniciando o seguidor de linha...")
    follower2.start()
    rospy.loginfo("Seguidor de linha finalizado. Executando a curva para a direita...")

    movement_sequence = [
        {'action': 'move_forward', 'duration': 1.5},
        {'action': 'center_all_sides'},
    ]

    #Curva
    move1 = MovementController()
    move1.start()
    move1.execute_movement_sequence(movement_sequence)

    while not rospy.is_shutdown():
        if move1.sequence_complete == 1:  
            move1.stop_controller()
            rospy.loginfo("Movimento concluído. Publicando feedback e encerrando.")
            move1.set_feedback_scheduler(1.0) 
            rospy.sleep(0.1)
            move1.stop_feedback()
            break


def seg_ALL5():

    #Giro inicial de 90°
    movement_sequence = [
        {'action': 'turn', 'direction': 'left', 'duration': 2.6},
    ]
    #Curva
    move = MovementController()
    move.start()
    move.use_camera = False
    move.execute_movement_sequence(movement_sequence)

    while not rospy.is_shutdown():
        if move.sequence_complete == 1:  
            rospy.loginfo("Movimento concluído. Publicando feedback e encerrando.")
            move.stop_controller()  # Parar controlador e timers
            move.stop_feedback()
            break


    movement_sequence = [
        {'action': 'move_forward', 'duration': 1.6},
        {'action': 'turn', 'direction': 'right', 'duration': 2.0},
    ]
    
    turn_side = 2  # Definir como virar à direit
    turn_at_intersection = 1  # Virar na 1ª interseção
    lost_line_turn = 2  # Virar à direita se perder a linha
    surpass_side = 1  # Fazer a ultrapassagem pela esquerda

    follower = LineFollower(turn_side, turn_at_intersection, lost_line_turn, surpass_side)

    rospy.loginfo("Iniciando o seguidor de linha...")
    follower.start()

    rospy.loginfo("Seguidor de linha finalizado. Executando a curva para a direita.")

    #Curva
    move = MovementController()
    move.start()
    move.execute_movement_sequence(movement_sequence)

    while not rospy.is_shutdown():
        if move.sequence_complete == 1:  
            rospy.loginfo("Movimento concluído. Publicando feedback e encerrando.")
            move.set_feedback_scheduler(1.0) 
            move.stop_feedback()
            move.stop_controller()
            break

    #Processo para andar até proximidade de X com a area de servico

def seg_ALL6():

    #Giro inicial de 90°
    movement_sequence = [
        {'action': 'turn', 'direction': 'left', 'duration': 2.6},
    ]
    #Curva
    move = MovementController()
    move.start()
    move.use_camera = False
    move.execute_movement_sequence(movement_sequence)

    while not rospy.is_shutdown():
        if move.sequence_complete == 1:  
            rospy.loginfo("Movimento concluído. Publicando feedback e encerrando.")
            move.stop_controller()  # Parar controlador e timers
            move.stop_feedback()
            break


    movement_sequence = [
        {'action': 'move_forward', 'duration': 1.2},
        {'action': 'turn', 'direction': 'left', 'duration': 2.5},
    ]
    
    turn_side = 2  # Definir como virar à direita
    turn_at_intersection = 1  # Virar na 1ª interseção
    lost_line_turn = 2  # Virar à direita se perder a linha
    surpass_side = 1  # Fazer a ultrapassagem pela esquerda

    follower = LineFollower(turn_side, turn_at_intersection, lost_line_turn, surpass_side)

    rospy.loginfo("Iniciando o seguidor de linha...")
    follower.start()

    rospy.loginfo("Seguidor de linha finalizado. Executando a curva para a esquerda.")

    #Curva
    move = MovementController()
    move.start()
    move.use_camera = False
    move.execute_movement_sequence(movement_sequence)

    while not rospy.is_shutdown():
        if move.sequence_complete == 1:  
            rospy.loginfo("Movimento concluído. Publicando feedback e encerrando.")
            move.stop_controller()
            break

    #Seguidor de linha, parte 2
    #FAzer a aproximação com a câmera

    move.set_feedback_scheduler(1.0) 
    move.stop_feedback()

def seg_41():

    #VIrar 180°
    movement_sequence = [
        {'action': 'turn', 'direction': 'right', 'duration': 4.4},
    ]

    #Curva
    move1 = MovementController()
    move1.start()
    move1.use_camera = False
    move1.execute_movement_sequence(movement_sequence)

     # Loop simples, sem rospy.Rate, para monitorar a variável de controle
    while not rospy.is_shutdown():
        if move1.sequence_complete == 1:  # Verifica se o movimento foi concluído
            rospy.loginfo("Movimento concluído. Publicando feedback e encerrando.")
            move1.set_feedback_scheduler(0.0)  # Publica o valor 1 no feedback
            move1.stop_feedback()
            move1.stop_controller()  # Parar controlador e timers
            break  
    ############  inicio do código #############
    movement_sequence = [
        {'action': 'move_forward', 'duration': 1.9},
        {'action': 'turn', 'direction': 'left', 'duration': 2.3},
    ]
    
    turn_side = 1  # Definir como virar à esquerda
    turn_at_intersection = 1  # Virar na 2ª interseção
    lost_line_turn = 0  # Virar à esquerda se perder a linha
    surpass_side = 1  # Fazer a ultrapassagem pela esquerda

    follower = LineFollower(turn_side, turn_at_intersection, lost_line_turn, surpass_side)

    rospy.loginfo("Iniciando o seguidor de linha...")
    follower.start()

    rospy.loginfo("Seguidor de linha finalizado. Executando a curva para a esquerda.")

    #Curva
    move = MovementController()
    move.start()
    move.execute_movement_sequence(movement_sequence)

    while not rospy.is_shutdown():
        if move.sequence_complete == 1:  
            move.stop_controller() 
            rospy.loginfo("Movimento concluído. Publicando feedback e encerrando.")
            move.stop_feedback()
            break

    rospy.sleep(3.0)

    turn_side = 1  # Definir como virar à esquerda
    turn_at_intersection = 1  # Virar na 2ª interseção
    lost_line_turn = 1  # Virar à esquerda se perder a linha
    surpass_side = 1  # Fazer a ultrapassagem pela esquerda

    follower2 = LineFollower(turn_side, turn_at_intersection, lost_line_turn, surpass_side)

    rospy.loginfo("Iniciando o seguidor de linha...")
    follower2.start()

    #Giro inicial de 90°
    movement_sequence = [
        {'action': 'move_forward', 'duration': 2.0},
        {'action': 'turn', 'direction': 'left', 'duration': 2.4},
    ]
    #Curva
    move = MovementController()
    move.start()
    move.use_camera = False
    move.execute_movement_sequence(movement_sequence)

    while not rospy.is_shutdown():
        if move.sequence_complete == 1:  
            rospy.loginfo("Movimento concluído. Publicando feedback e encerrando.")
            move.set_feedback_scheduler(1.0)  # Publica o valor 1 no feedback
            rospy.sleep(0.1)
            move.stop_controller()  # Parar controlador e timers
            move.stop_feedback()
            break
        rospy.loginfo("Seguidor de linha finalizado. Executando a curva para a direita...")

    rospy.loginfo("Seguidor de linha finalizado. Executando a curva para a direita...")


def seg_31():
    movement_sequence = [
        #{'action': 'move_back', 'duration': 0.5},
        {'action': 'turn', 'direction': 'right', 'duration': 2.6},
    ]

    #Curva
    move1 = MovementController()
    move1.start()
    move1.use_camera = False
    move1.execute_movement_sequence(movement_sequence)

     # Loop simples, sem rospy.Rate, para monitorar a variável de controle
    while not rospy.is_shutdown():
        if move1.sequence_complete == 1:  # Verifica se o movimento foi concluído
            rospy.loginfo("Movimento concluído. Publicando feedback e encerrando.")
            move1.set_feedback_scheduler(0.0)  # Publica o valor 1 no feedback
            move1.stop_feedback()
            move1.stop_controller()  # Parar controlador e timers
            break  
    ############  inicio do código #############
    movement_sequence = [
        {'action': 'move_forward', 'duration': 2.3},
        {'action': 'turn', 'direction': 'right', 'duration': 2.2},
    ]
    
    turn_side = 2  # Definir como virar à esquerda
    turn_at_intersection = 1  # Virar na 2ª interseção
    lost_line_turn = 2  # Virar à esquerda se perder a linha
    surpass_side = 1  # Fazer a ultrapassagem pela esquerda

    follower = LineFollower(turn_side, turn_at_intersection, lost_line_turn, surpass_side)

    rospy.loginfo("Iniciando o seguidor de linha...")
    follower.start()

    rospy.loginfo("Seguidor de linha finalizado. Executando a curva para a esquerda.")

    #Curva
    move = MovementController()
    move.start()
    move.execute_movement_sequence(movement_sequence)

    while not rospy.is_shutdown():
        if move.sequence_complete == 1:  
            rospy.loginfo("Movimento concluído. Publicando feedback e encerrando.")
            move.stop_controller()  # Parar controlador e timers
            move.stop_feedback()
            break

    rospy.sleep(3.0)

    turn_side = 1  # Definir como virar à esquerda
    turn_at_intersection = 1  # Virar na 2ª interseção
    lost_line_turn = 0  # Virar à esquerda se perder a linha
    surpass_side = 1  # Fazer a ultrapassagem pela esquerda

    follower2 = LineFollower(turn_side, turn_at_intersection, lost_line_turn, surpass_side)

    rospy.loginfo("Iniciando o seguidor de linha...")
    follower2.start()

    #Giro inicial de 90°
    movement_sequence = [
        {'action': 'move_forward', 'duration': 2.0},
        {'action': 'turn', 'direction': 'left', 'duration': 2.4},
    ]
    #Curva
    move = MovementController()
    move.start()
    move.use_camera = False
    move.execute_movement_sequence(movement_sequence)

    while not rospy.is_shutdown():
        if move.sequence_complete == 1:  
            rospy.loginfo("Movimento concluído. Publicando feedback e encerrando.")
            move.set_feedback_scheduler(1.0)  # Publica o valor 1 no feedback
            rospy.sleep(0.1)
            move.stop_controller()  # Parar controlador e timers
            move.stop_feedback()
            break
        rospy.loginfo("Seguidor de linha finalizado. Executando a curva para a direita...")

    rospy.loginfo("Seguidor de linha finalizado. Executando a curva para a direita...")



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
    elif goal_value == 41:
        rospy.loginfo("Executando o segmento 41 (seg_41).")
        seg_41()
    elif goal_value == 31:
        seg_31()
        rospy.loginfo("Executando o segmento 31 (seg_31).")
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

