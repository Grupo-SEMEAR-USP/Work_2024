#!/usr/bin/env python
import rospy
import smach
import smach_ros
import time
# import RPi.GPIO as GPIO
from robot_actions.msg import MoveToShelfAction, MoveToShelfGoal

blocos = 2

def wait_for_signal():
    rospy.loginfo("Aguardando sinal para iniciar...")

    input()

    #GPIO.wait_for_edge(12, GPIO.RISING)  # Aguarda uma transição de baixo para alto

def main():
    rospy.init_node('robot_task_scheduler')

    # Criar a máquina de estados com userdata inicial
    sm = smach.StateMachine(outcomes=['task_completed', 'task_failed'])
    sm.userdata.shelf_id = 1 
    sm.userdata.deposit_id = 2
    sm.userdata.final_id = 3

    # Vizualizador das tasks
    sis = smach_ros.IntrospectionServer('robot_task_scheduler_server', sm, '/SM_ROOT_ROBOT_TASK_SCHEDULER')
    sis.start()

    with sm:
        # Estado para iniciar o robô
        @smach.cb_interface(outcomes=['robot_ok'])
        def init_robot_cb(userdata):
            rospy.loginfo("Iniciando o robô...")

            wait_for_signal()  # Aguarda sinal do GPIO para começar

            return 'robot_ok'

        smach.StateMachine.add('INIT_ROBOT', smach.CBState(init_robot_cb), transitions={'robot_ok':'MOVE_TO_SHELF'})

        # Estado para mover o robô até a prateleira
        smach.StateMachine.add('MOVE_TO_SHELF',
                               smach_ros.SimpleActionState('move_to_shelf', 
                                                           MoveToShelfAction,
                                                           goal_slots=['shelf_id']),
                               transitions={'succeeded':'PICK_BLOCKS', 'aborted':'task_failed', 'preempted':'task_failed'},
                               remapping={'shelf_id':'shelf_id'})

        # Estado para pegar blocos
        @smach.cb_interface(outcomes=['catch_block'])
        def pick_blocks_cb(userdata):
            global blocos
            rospy.loginfo("Pegando blocos...")
            time.sleep(3)
            blocos -= 1  # Decrementa a contagem de blocos
            return 'catch_block'

        smach.StateMachine.add('PICK_BLOCKS', smach.CBState(pick_blocks_cb), transitions={'catch_block':'MOVE_TO_DEPOSIT'})

        # Estado para mover o robô até o depósito
        smach.StateMachine.add('MOVE_TO_DEPOSIT',
                               smach_ros.SimpleActionState('move_to_shelf',
                                                           MoveToShelfAction,
                                                           goal_slots=['shelf_id']),
                               transitions={'succeeded':'DEPOSIT_BLOCKS', 'aborted':'task_failed', 'preempted':'task_failed'},
                               remapping={'shelf_id':'deposit_id'})

        # Estado para depositar blocos
        @smach.cb_interface(outcomes=['deposited_blocks', 'no_blocks'])
        def deposit_blocks_cb(userdata):
            global blocos
            rospy.loginfo("Depositando blocos...")
            time.sleep(3)
            if blocos == 0:
                return 'no_blocks'
            return 'deposited_blocks'

        smach.StateMachine.add('DEPOSIT_BLOCKS', smach.CBState(deposit_blocks_cb), transitions={'deposited_blocks':'MOVE_TO_SHELF', 'no_blocks':'MOVE_TO_FINAL'})

        # Estado final para mover o robô até a localidade final
        smach.StateMachine.add('MOVE_TO_FINAL',
                               smach_ros.SimpleActionState('move_to_shelf',
                                                           MoveToShelfAction,
                                                           goal_slots=['shelf_id']),
                               transitions={'succeeded':'task_completed', 'aborted':'task_failed', 'preempted':'task_failed'},
                               remapping={'shelf_id':'final_id'})

    # Executar a máquina de estados
    outcome = sm.execute()

    # Aguardar até que ROS seja desligado, então pare o servidor de introspecção
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
