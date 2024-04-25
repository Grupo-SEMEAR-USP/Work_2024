#!/usr/bin/env python3
import rospy
import smbus
import struct
from robot_control.msg import encoder_data, velocity_data
from time import sleep
import threading

# Constantes
ESP32_ADDRESS_FRONT = 0x08
ESP32_ADDRESS_REAR = 0x09
I2C_BUS = 1
REG_ADDRESS = 10
MAX_WHEEL_VELOCITY = 2000  # Velocidade máxima permitida em unidades do sensor

encoder_data_global = [0, 0, 0, 0]

class I2CCommunication:
    def __init__(self, device_address):
        self.wheel_velocities = [0, 0]
        self.wheel_indices = 0 if device_address == ESP32_ADDRESS_FRONT else 1
        self.i2c = smbus.SMBus(I2C_BUS)
        self.device_address = device_address
        self.pub_encoder = rospy.Publisher('/encoder_data', encoder_data, queue_size=10)
        self.sub_joints = rospy.Subscriber('/velocity_command', velocity_data, self.joints_callback)
        self.encoder_msg = encoder_data()

        # Thread de atualização
        self.rate = rospy.Rate(10)
        threading.Thread(target=self.update).start()

    def validate_velocity(self, velocity):
        if abs(velocity) > MAX_WHEEL_VELOCITY:
            rospy.logwarn(f"Velocidade {velocity} fora do limite! Limitando a {MAX_WHEEL_VELOCITY}.")
            return max(min(velocity, MAX_WHEEL_VELOCITY), -MAX_WHEEL_VELOCITY)
        return velocity

    def read_data(self):
        try:
            data = self.i2c.read_i2c_block_data(self.device_address, REG_ADDRESS, 8)
            value_right = struct.unpack('!i', bytes(data[:4]))
            value_left = struct.unpack('!i', bytes(data[4:]))

            index = 0 if self.wheel_indices == 0 else 2

            rospy.loginfo(f'Valor lido: {value_left[0]}, {value_right[0]}')

            encoder_data_global[index] = value_left[0]
            encoder_data_global[index + 1] = value_right[0]

            self.encoder_msg.front_left_encoder_data = encoder_data_global[0] / 1000
            self.encoder_msg.front_right_encoder_data = encoder_data_global[1] / 1000
            self.encoder_msg.rear_left_encoder_data = encoder_data_global[2] / 1000
            self.encoder_msg.rear_right_encoder_data = encoder_data_global[3] / 1000

            self.pub_encoder.publish(self.encoder_msg)

        except smbus.SMBusError as e:
            rospy.logerr(f"Erro na leitura I2C: {str(e)}")
            sleep(0.5)  # Espera antes de tentar novamente
            self.read_data()  # Tenta ler novamente

    def write_data(self):
        try:
            velocities = [self.validate_velocity(v) for v in self.wheel_velocities]

            data = struct.pack('!ii', *velocities)

            self.i2c.write_i2c_block_data(self.device_address, REG_ADDRESS, list(data))

            rospy.loginfo(f'Valores enviados: {velocities}')

        except smbus.SMBusError as e:
            rospy.logerr(f"Erro na escrita I2C: {str(e)}")
            
            sleep(0.5)  # Espera antes de tentar novamente
            self.write_data()  # Tenta escrever novamente

    def joints_callback(self, msg):
        if msg is not None:
            index = 0 if self.wheel_indices == 0 else 1
            self.wheel_velocities[0] = self.validate_velocity(int(msg.velocity[index * 2] * 1000))
            self.wheel_velocities[1] = self.validate_velocity(int(msg.velocity[index * 2 + 1] * 1000))

    def update(self):
        while not rospy.is_shutdown():
            self.read_data()
            self.write_data()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node('i2c_master', anonymous=True)
        right_i2c_communication = I2CCommunication(ESP32_ADDRESS_FRONT)
        left_i2c_communication = I2CCommunication(ESP32_ADDRESS_REAR)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
