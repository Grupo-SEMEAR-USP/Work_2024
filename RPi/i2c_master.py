#!/usr/bin/env python3
import rospy
import smbus
import struct
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from robot_control.msg import encoder_data, velocity_data
from time import sleep
import threading

# Autores: Matheus Paiva Angarola e William

# Constantes
ESP32_ADDRESS_LEFT = 0x08  # Endereço do dispositivo ESP32 esquerdo no barramento I2C
ESP32_ADDRESS_RIGHT = 0x09  # Endereço do dispositivo ESP32 direito no barramento I2C
I2C_BUS = 1  # Número do barramento I2C no Raspberry Pi
REG_ADDRESS = 10  # Endereço de registro (offset) a ser usado pelo PID

# Classe para comunicação I2C
class I2CCommunication:
    # "Construtor" da classe, definindo seus atributos principais
    def __init__(self, device_address):

        self.wheel_velocities = [0, 0]  # Inicializa as velocidades das duas rodas como zero

        if device_address == ESP32_ADDRESS_LEFT:
            self.wheel_indices = 0 # Indice front
        elif device_address == ESP32_ADDRESS_RIGHT:
            self.wheel_indice = 1  # Indice rear

        self.i2c = smbus.SMBus(I2C_BUS)  # Define o barramento que será usado na comunicação
        self.device_address = device_address  # Define o endereço da ESP32 ao qual queremos nos comunicar

        # Cria um objeto de publicação para enviar dados para o tópico ROS
        self.pub_encoder = rospy.Publisher('/encoder_data', encoder_data, queue_size=10)
        self.sub_joints = rospy.Subscriber('/velocity_command', velocity_data, self.joints_callback)

        self.encoder_msg = encoder_data()

        if self.wheel_indice == 0:
            self.encoder_msg.front_left_encoder_data = 0
            self.encoder_msg.front_right_encoder_data = 0
        else:
            self.encoder_msg.rear_left_encoder_data = 0
            self.encoder_msg.rear_right_encoder_data = 0

        self.thread = threading.Thread(target=self.update)  # Cria uma nova thread para a função update
        self.thread.start()  # Inicia a execução da thread

    def read_data(self):
        try:
            data = self.i2c.read_i2c_block_data(self.device_address, REG_ADDRESS, 8)  # Faz a leitura da ESP32
            value_right, value_left = struct.unpack('!ii', bytes(data[:3]), bytes(data[3:]))  # Desempacota as informações recebidas

            rospy.loginfo(f'Valor lido: {value_left}, {value_right}')

            if self.wheel_indice == 0:
                self.encoder_msg.front_left_encoder_data = value_left
                self.encoder_msg.front_right_encoder_data = value_right
            else:
                self.encoder_msg.rear_left_encoder_data = value_left
                self.encoder_msg.rear_right_encoder_data = value_right

            self.pub_encoder.publish(self.encoder_msg)

        except Exception as e:
            rospy.logerr(f"Erro na leitura: {str(e)}")
            return None

    def write_data(self):
        try:
            data = struct.pack('!ii', self.wheel_velocities[0], self.wheel_velocities[1])  # Empacota os valores das velocidades das rodas
            self.i2c.write_i2c_block_data(self.device_address, REG_ADDRESS, list(data))  # Escreve valores para a ESP32

            rospy.loginfo(f'Valores enviados: {self.wheel_velocities}')

        except Exception as e:
            rospy.logerr(f"Erro na escrita: {str(e)}")
            return None

    def joints_callback(self, msg):
        if msg is not None:
            if self.wheel_indices == 0:
                self.wheel_velocities[0] = int(msg.front_left_wheel * 100)
                self.wheel_velocities[1] = int(msg.front_right_wheel * 100)
            else:
                self.wheel_velocities[0] = int(msg.rear_left_wheel * 100)
                self.wheel_velocities[1] = int(msg.rear_right_wheel * 100)

    def update(self):
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown():
            self.read_data()
            self.write_data()

            rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node('i2c_master', anonymous=True)
        left_i2c_communication = I2CCommunication(ESP32_ADDRESS_LEFT)
        right_i2c_communication = I2CCommunication(ESP32_ADDRESS_RIGHT)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
