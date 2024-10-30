#Defining the PID parameters
KP = 0.4
KI = 0
KD = 0

#Defining the maximum and minimum values for the control signal
#As the control signal is the PWM value:
PWM_CHANNEL_RESOLUTION = 10
MAX_SIG = 2**PWM_CHANNEL_RESOLUTION - 1
MIN_SIG = -2**PWM_CHANNEL_RESOLUTION - 1
MAX_INTEGRAL = 5
MIN_INTEGRAL = 0.1

#Defining the Period of the control loop
PERIOD = 5

INITIAL_VEL = 0.0