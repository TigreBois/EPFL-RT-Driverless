import socket
import sys
import pickle
import time
import tcplib
import SLAM

IP_PORT_IN_PERCEPTION =  10000
IP_PORT_IN_MOTION =  10001
IP_PORT_OUT = 10002
IP_ADDR = 'localhost'
MY_MODULE_NAME = 'SLAM'

print('INFO:', MY_MODULE_NAME, 'starting.')

######################################################
# INITIALIZE YOUR MODULE HERE
######################################################

# Create a TCP/IP socket for the perception input
sock_input_perception = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address_in_perception = (IP_ADDR, IP_PORT_IN_PERCEPTION)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for input.'.format(*server_address_in))
sock_input_perception.connect(server_address_in)

# Create a TCP/IP socket for the motion estimation input
sock_input_motion = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address_in_motion = (IP_ADDR, IP_PORT_IN_MOTION)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for input.'.format(*server_address_in))
sock_input_motion.connect(server_address_in_motion)

# Create a TCP/IP socket for the output
sock_output = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address_out = (IP_ADDR, IP_PORT_OUT)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for output.'.format(*server_address_out))
sock_output.connect(server_address_out)

module_running = True

try:
    while module_running:

        ######################################################
        # WHEN YOU ARE READY, START READING INCOMING DATA
        # TO PROCESS IT. THIS FUNCTION CALL IS BLOCKING,
        # IT WILL WAIT UNTIL THERE'S DATA. THE DATA WILL 
        # BE WAITING AND PILE UP UNTIL YOU READ IT.
        ######################################################
        cone_data   = tcplib.receiveData(sock_input_perception)
        motion_data = tcplib.receiveData(sock_input)
        built_map   = SLAM.main(cone_data, motion_data)

        ######################################################
        # PERFORM SOME CALCULATIONS.
        # WHEN YOU GET A RESULT YOU WISH TO SEND,
        # CONTINUE WITH THE CODE BELOW
        ######################################################
        tcplib.sendData(sock_output, built_map)

        ######################################################
        # IF YOU'RE DONE HANDLING THE DATA, THE LOOP
        # WILL TAKE YOU BACK TO WAITING ON INCOMING DATA.
        # IF YOUR MODULE ENCOUNTERS AN ERROR OR NEEDS TO 
        # TERMINATE, CONTINUE WITH THE CODE BELOW
        ######################################################
        # Need to figure out the terminating condition, but for let's pretend it never stops
        #if len(built_map) > 20:
        #    module_running = False

finally:
    sock_input.close()
    sock_output.close()