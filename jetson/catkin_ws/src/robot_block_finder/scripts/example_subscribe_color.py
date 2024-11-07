#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json

class BlockColorSubscriber:
    def __init__(self):
        # Inicializa o nó ROS
        rospy.init_node('block_color_subscriber', anonymous=True)

        # Inscreve-se no tópico que contém o dicionário de blocos detectados em JSON
        rospy.Subscriber('/detected_blocks', String, self.callback)

    def callback(self, msg):
        # Converte a mensagem JSON de volta para um dicionário
        try:
            detected_blocks = json.loads(msg.data)
            rospy.loginfo(f"Blocos detectados: {detected_blocks}")
            # Aqui você pode adicionar qualquer lógica que precise usar o dicionário `detected_blocks`
        except json.JSONDecodeError as e:
            rospy.logerr(f"Erro ao decodificar JSON: {e}")

    def run(self):
        rospy.spin()  # Mantém o nó em execução para receber mensagens

if __name__ == '__main__':
    try:
        subscriber = BlockColorSubscriber()
        subscriber.run()
    except rospy.ROSInterruptException:
        pass
