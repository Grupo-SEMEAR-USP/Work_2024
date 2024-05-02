#!/usr/bin/env python3
import definitions
import matplotlib.pyplot as plt
import numpy as np
import time
'''PID controller for a robot using PWM control signal'''

#How to transform RPM in RADs/sec = 1RPM = 0.10472 rads/secs 

#Function to get the reference velocity
def get_reference_velocity():
    vel_ref = float(input("Enter the reference velocity (RPM): "))
    return vel_ref

def get_current_velocity():
    #Pega os valores da comunicação serial
    vel = float(input("Enter the RPM data: "))
    #vel = RPML
    #botar a comunicação aqui
    return vel

#Main function, reponsible for running the control loop
def control_main():
    c_velocity_keep = []
    #Velocity message and reference velocity
    velocity_ref = get_reference_velocity()
    current_velocity = get_current_velocity()

    c_velocity_keep.append(current_velocity)

    total_time = 0
    current_time = past_time = time.time()

    while (True):
        while ((current_time - past_time) < definitions.PERIOD):
            #Calculate the error
            error = velocity_ref - current_velocity
            print(f'Error: {error}')

            current_velocity = get_reference_velocity()
            c_velocity_keep.append(current_velocity)

            print("Current velocity: ", current_velocity)

            current_time = time.time()
            print(f'Period: {current_time - past_time}')


            time.sleep(0.3)

        #Plotando o gráfico de conversão:
        total_time += definitions.PERIOD
        x = np.linspace(0, total_time, len(c_velocity_keep))
        ref = velocity_ref*np.ones_like(x)
        plt.plot(x, c_velocity_keep, label='PWM')
        plt.plot(x, ref, label='Referência')
        plt.xlabel('Tempo (seg)')
        plt.ylabel('Velocidade Linear (M/s)')
        plt.title('Resposta do Controlador PID')
        plt.legend()
        plt.grid(True)

        plt.show()

        velocity_ref = get_reference_velocity()
        current_time = past_time = time.time()  


if __name__ == '__main__':
    control_main()
   