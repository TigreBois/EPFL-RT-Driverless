import socket
import sys
import pickle
import time
import tcplib
import mpc_car_model
import math as mt

PATH_PLANNING_PORT = 10050
MOTION_EST_PORT = 10051
VEHICLE_PORT = 10052

IP_ADDR = 'localhost'
BUFF_SIZE = 2**5    # has to be a power of 2
MY_MODULE_NAME = 'mpc_module' # Please enter here an arbitrary name for your code, which will help in logging

print('INFO:', MY_MODULE_NAME, 'starting.')

######################################################
#   MPC MODULE DECALRATION
######################################################
mpc_module = mpc_car_model.MPC_car_model()

#Socket for path planning
sock_path = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address_in = (IP_ADDR, PATH_PLANNING_PORT)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for input.'.format(*server_address_in))
sock_path.connect(server_address_in)

#Socket for motion estimator
sock_motion = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address_in = (IP_ADDR, MOTION_EST_PORT)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for input.'.format(*server_address_in))
sock_motion.connect(server_address_in)


# Socket for the simulation
sock_output = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address_out = (IP_ADDR, VEHICLE_PORT)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for output.'.format(*server_address_out))
sock_output.connect(server_address_out)


module_running = True

try:
    iter = 0
    while module_running:

        # Since for now we receive path and state at really fequence
        # We assume that at each iteration we will update our current paramaters
        # That won't be the case at the end we should only update our aprameters if we get new ones 
        new_state = tcplib.receiveData(sock_motion)
        new_path = mpc_car_model.local_to_global(tcplib.receiveData(sock_path), new_state)


        # Computation of the output given the new input (asked A. Deherse to be sure about argument order)
        # new_state = (new_state[0], new_state[1], new_state[5])
        # previous_state = (new_state[3], new_state[5])
        # #mpc_module.previous_state = previous_state
        mpc_module.acquire_path(new_path)
        mpc_module.set_state(new_state)
        output = mpc_module.run_MPC()

        #Sending to new command to the car
        tcplib.sendData(sock_output, output)

        if False :
            module_running = False

finally:
    sock_path.close()
    sock_motion.close()