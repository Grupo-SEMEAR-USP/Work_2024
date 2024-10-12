#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import time

feedback_received = False
feedback_success = False

def feedback_callback(msg):
    global feedback_received, feedback_success
    feedback_received = True
    if msg.data == 1:
        feedback_success = True
    else:
        feedback_success = False

def execute_task(task_name, feedback_topic):
    global feedback_received, feedback_success
    success = False

    while not success:
        feedback_received = False
        
        rospy.Subscriber(feedback_topic, Int32, feedback_callback)
        
        rospy.loginfo(f"Executando tarefa: {task_name}")
        
        while not feedback_received:
            rospy.sleep(0.1)
            if rospy.is_shutdown():
                return False

        if feedback_success:
            rospy.loginfo(f"Tarefa '{task_name}' concluída com sucesso.\n")
            success = True
        else:
            rospy.logwarn(f"Tarefa '{task_name}' falhou. Tentando novamente...\n")

def setup_robo():
    # Tempo para o robô se preparar/estabilizar
    time.sleep(2)

    execute_task("setup do robô", "/setup_feedback")

def alinhar_ate_mesa():
    execute_task("alinhar até a mesa", "/alinhar_feedback")

def identificar_blocos():
    execute_task("identificar blocos", "/identificar_feedback")

def pegar_blocos_vermelhos():
    execute_task("pegar blocos vermelhos", "/pegar_vermelhos_feedback")

def ir_ao_deposito_vermelho():
    execute_task("ir ao depósito vermelho", "/deposito_vermelho_feedback")

def guardar_blocos_vermelhos():
    execute_task("guardar blocos vermelhos", "/guardar_vermelhos_feedback")

def pegar_blocos_azuis():
    execute_task("pegar blocos azuis", "/pegar_azuis_feedback")

def ir_ao_deposito_azul():
    execute_task("ir ao depósito azul", "/deposito_azul_feedback")

def guardar_blocos_azuis():
    execute_task("guardar blocos azuis", "/guardar_azuis_feedback")

def ir_para_area_final():
    execute_task("ir para a área final", "/area_final_feedback")

def finalizar_robo():
    execute_task("finalizar robô", "/finalizar_feedback")

def scheduler():
    rospy.init_node('scheduler_node', anonymous=True)
    
    tasks = [
        setup_robo,
        alinhar_ate_mesa,
        identificar_blocos,
        pegar_blocos_vermelhos,
        ir_ao_deposito_vermelho,
        guardar_blocos_vermelhos,
        alinhar_ate_mesa,
        pegar_blocos_azuis,
        ir_ao_deposito_azul,
        guardar_blocos_azuis,
        ir_para_area_final,
        finalizar_robo
    ]
    
    for task in tasks:
        if rospy.is_shutdown():
            break
        task()

if __name__ == '__main__':
    try:
        scheduler()
    except rospy.ROSInterruptException:
        pass
