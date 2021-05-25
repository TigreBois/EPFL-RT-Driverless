import socket
import sys
import pickle
import time
import tcplib
import numpy as np
from scipy.spatial import Delaunay
from scipy.interpolate import interp1d
from numpy import arccos
from numpy.linalg import norm
from math import pi
from pathplanning.src.MainCode import middle_path
import pathplanning.src.MainCode
from pathplanning.src.MainCode import get_angles, mid_triangle, sort_norm

# Beginning of my code

max_iter = 10  # Number of middle points to be selected
point_step = 0.5  # Space between points in output in meters
origin = np.array([0, 0])  # origin = car position
rad_to_deg = 180 / pi


# Compute the middle path between the cones
def get_path(points):
    # separate blue and yellow cones
    mid = int(len(points) / 2)
    return middle_path(points[mid:], points[:mid])



# Beginning of Wrapper

IP_PORT_IN = 10040  # input form SLAM
IP_PORT_OUT = 10041  # output to control
IP_ADDR = 'localhost'
BUFF_SIZE = 2 ** 5  # has to be a power of 2
MY_MODULE_NAME = 'path_planning'  # Please enter here an arbitrary name for your code, which will help in logging

print('INFO:', MY_MODULE_NAME, 'starting.')

######################################################
# INITIALIZE YOUR MODULE HERE
######################################################

# Create a TCP/IP socket for the input
sock_input = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address_in = (IP_ADDR, IP_PORT_IN)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for input.'.format(*server_address_in))
sock_input.connect(server_address_in)

# Create a TCP/IP socket for the output
sock_output = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address_out = (IP_ADDR, IP_PORT_OUT)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for output.'.format(*server_address_out))
sock_output.connect(server_address_out)

module_running = True

try:
    iter = 0
    while module_running:

        ######################################################
        # WHEN YOU ARE READY, START READING INCOMING DATA
        # TO PROCESS IT. THIS FUNCTION CALL IS BLOCKING,
        # IT WILL WAIT UNTIL THERE'S DATA. THE DATA WILL
        # BE WAITING AND PILE UP UNTIL YOU READ IT.
        ######################################################
        msg = tcplib.receiveData(sock_input)
        print('message:', msg)

        path, yellow, blue = get_path(msg)

        ######################################################
        # PERFORM SOME CALCULATIONS.
        # WHEN YOU GET A RESULT YOU WISH TO SEND,
        # CONTINUE WITH THE CODE BELOW
        ######################################################
        my_results = path
        tcplib.sendData(sock_output, my_results)

        ######################################################
        # IF YOU'RE DONE HANDLING THE DATA, THE LOOP
        # WILL TAKE YOU BACK TO WAITING ON INCOMING DATA.
        # IF YOUR MODULE ENCOUNTERS AN ERROR OR NEEDS TO
        # TERMINATE, CONTINUE WITH THE CODE BELOW
        ######################################################
        if False:
            module_running = False

finally:
    sock_input.close()
    sock_output.close()
