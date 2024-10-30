#!/usr/bin/env python3

from line_follower import LineFollower
from movement_controller import MovementController
import rospy

def seg_12():
    rospy.init_node('robot_line_follower_node', anonymous=True)

    movement_sequence = [
        {'action': 'move_forward', 'duration': 1.8},
        {'action': 'turn', 'direction': 'left', 'duration': 2.0},
    ]
    
    turn_side = 1  # Definir como virar à esquerda
    turn_at_intersection = 1  # Virar na 2ª interseção
    lost_line_turn = 0  # Virar à esquerda se perder a linha

    #Seguidor de linha, parte 1
    follower = LineFollower(turn_side, turn_at_intersection, lost_line_turn)

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
            move.set_feedback_scheduler(1.0)  # Publica o valor 1 no feedback
            rospy.sleep(0.1)
            move.stop_controller()  # Parar controlador e timers
            move.stop_feedback()
            break

    #Processo de andar at'proximidade X

def seg_13():
    rospy.init_node('robot_line_follower_node', anonymous=True)
    movement_sequence = [
        {'action': 'move_forward', 'duration': 2.0},
        {'action': 'turn', 'direction': 'left', 'duration': 2.1},
    ]
    
    turn_side = 1  # Definir como virar à esquerda
    turn_at_intersection = 2 # Virar na 2ª interseção
    lost_line_turn = 0  # Virar à esquerda se perder a linha

    #Seguidor de linha, parte 1
    follower = LineFollower(turn_side, turn_at_intersection, lost_line_turn)

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
            move.set_feedback_scheduler(0.0)  # Publica o valor 1 no feedback
            move.stop_controller()  # Parar controlador e timers
            #move.stop_feedback()
            break
    #Seguidor de linha, parte 2

    rospy.sleep(4.0)

    movement_sequence = [
        {'action': 'move_forward', 'duration': 2.2},
        {'action': 'turn', 'direction': 'right', 'duration': 2.0},
    ]

    turn_side = 2  # Definir como virar à esquerda
    turn_at_intersection = 1  # Virar na 2ª interseção
    lost_line_turn = 0  # Virar à esquerda se perder a linha


    follower2 = LineFollower(turn_side, turn_at_intersection, lost_line_turn)

    rospy.loginfo("Iniciando o seguidor de linha...")
    follower2.start()
    rospy.loginfo("Seguidor de linha finalizado. Executando a curva para a direita...")

    #Curva
    move1 = MovementController()
    move1.start()
    move.use_camera = False
    move1.execute_movement_sequence(movement_sequence)

     # Loop simples, sem rospy.Rate, para monitorar a variável de controle
    while not rospy.is_shutdown():
        if move1.sequence_complete == 1:  # Verifica se o movimento foi concluído
            rospy.loginfo("Movimento concluído. Publicando feedback e encerrando.")
            move1.set_feedback_scheduler(1.0)  # Publica o valor 1 no feedback
            move1.stop_feedback()
            move1.stop_controller()  # Parar controlador e timers
            break  

    # Encerrando o ROS após o fim do movimento
    rospy.signal_shutdown("Movimento finalizado, encerrando o ROS.")



def seg_14():
    rospy.init_node('robot_controller', anonymous=True)

    movement_sequence = [
        {'action': 'move_forward', 'duration': 0.7},
        {'action': 'turn', 'direction': 'right', 'duration': 2.1},
    ]
    
    turn_side = 1  # Definir como virar à esquerda
    turn_at_intersection = 2  # Virar na 2ª interseção
    lost_line_turn = 2  # Virar à direita se perder a linha

    #Seguidor de linha, parte 1
    follower = LineFollower(turn_side, turn_at_intersection, lost_line_turn)

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

    turn_side = 1  # Definir como virar à esquerda
    turn_at_intersection = 2  # Virar na 2ª interseção
    lost_line_turn = 0  # Andar rto até ahar a linha

    follower2 = LineFollower(turn_side, turn_at_intersection, lost_line_turn)

    rospy.loginfo("Iniciando o seguidor de linha...")
    follower2.start()
    rospy.loginfo("Seguidor de linha finalizado. Executando a curva para a direita...")

    move.set_feedback_scheduler(1.0) 
    move.stop_feedback()
    rospy.signal_shutdown("Movimento finalizado, encerrando o ROS.")

def seg_ALL5():
    rospy.init_node('robot_controller', anonymous=True)

    movement_sequence = [
        {'action': 'move_forward', 'duration': 1.6},
        {'action': 'turn', 'direction': 'right', 'duration': 2.0},
    ]
    
    turn_side = 2  # Definir como virar à direit
    turn_at_intersection = 1  # Virar na 1ª interseção
    lost_line_turn = 2  # Virar à direita se perder a linha

    #Seguidor de linha, parte 1
    follower = LineFollower(turn_side, turn_at_intersection, lost_line_turn)

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
    rospy.init_node('robot_controller', anonymous=True)

    movement_sequence = [
        {'action': 'move_forward', 'duration': 1.2},
        {'action': 'turn', 'direction': 'left', 'duration': 2.5},
    ]
    
    turn_side = 2  # Definir como virar à direita
    turn_at_intersection = 1  # Virar na 1ª interseção
    lost_line_turn = 2  # Virar à direita se perder a linha

    #Seguidor de linha, parte 1
    follower = LineFollower(turn_side, turn_at_intersection, lost_line_turn)

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
    rospy.signal_shutdown("Movimento finalizado, encerrando o ROS.")


"""     Implementação das funções de Volta    """

def seg_41():
    rospy.init_node('robot_line_follower_node', anonymous=True)
    movement_sequence = [
        {'action': 'move_forward', 'duration': 2.0},
        {'action': 'turn', 'direction': 'left', 'duration': 2.1},
    ]
    
    turn_side = 1  # Definir como virar à esquerda
    turn_at_intersection = 1 # Virar na 2ª interseção
    lost_line_turn = 0  # Virar à esquerda se perder a linha

    #Seguidor de linha, parte 1
    follower = LineFollower(turn_side, turn_at_intersection, lost_line_turn)

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
            move.set_feedback_scheduler(0.0)  # Publica o valor 1 no feedback
            move.stop_controller()  # Parar controlador e timers
            #move.stop_feedback()
            break
    #Seguidor de linha, parte 2

    rospy.sleep(4.0)

    movement_sequence = [
        {'action': 'move_forward', 'duration': 2.2},
        {'action': 'turn', 'direction': 'right', 'duration': 2.0},
    ]

    turn_side = 2  # Definir como virar à esquerda
    turn_at_intersection = 1  # Virar na 2ª interseção
    lost_line_turn = 0  # Virar à esquerda se perder a linha


    follower2 = LineFollower(turn_side, turn_at_intersection, lost_line_turn)

    rospy.loginfo("Iniciando o seguidor de linha...")
    follower2.start()
    rospy.loginfo("Seguidor de linha finalizado. Executando a curva para a direita...")

    #Curva
    move1 = MovementController()
    move1.start()
    move.use_camera = False
    move1.execute_movement_sequence(movement_sequence)

     # Loop simples, sem rospy.Rate, para monitorar a variável de controle
    while not rospy.is_shutdown():
        if move1.sequence_complete == 1:  # Verifica se o movimento foi concluído
            rospy.loginfo("Movimento concluído. Publicando feedback e encerrando.")
            move1.set_feedback_scheduler(1.0)  # Publica o valor 1 no feedback
            move1.stop_feedback()
            move1.stop_controller()  # Parar controlador e timers
            break  

    # Encerrando o ROS após o fim do movimento
    rospy.signal_shutdown("Movimento finalizado, encerrando o ROS.")

# def main():
#     seg_13()

# if __name__ == '__main__':
#     main()
