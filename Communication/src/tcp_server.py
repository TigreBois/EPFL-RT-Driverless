import socket
import sys
import pickle
import tcplib
import time
import threading

############################################################
# GENERAL
############################################################
IP_ADDR = 'localhost'
BUFF_SIZE = 2**12               # Ideally a power of 2

IP_PORT_PERCEPTION_OUT1 =    10000
IP_PORT_SLAM_IN1 =           10001

IP_PORT_ME_OUT1 =            10002

############################################################
# PERCEPTION SOCKET(S)
############################################################
socket_perception_out = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_perception_out = (IP_ADDR, IP_PORT_PERCEPTION_OUT1)
print('Perception Out starting up on \'{}\' port: {}'.format(*server_perception_out))
socket_perception_out.bind(server_perception_out)


############################################################
# SLAM SOCKET(S)
############################################################
socket_slam_in = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_slam_in = (IP_ADDR, IP_PORT_SLAM_IN1)
print('SLAM IN starting up on \'{}\' port: {}'.format(*server_slam_in))
socket_slam_in.bind(server_slam_in)


############################################################
# MOTION ESTIMATION SOCKET(S)
############################################################
"""
socket_me_in = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_me_in = (IP_ADDR, IP_PORT_ME_OUT1)
print('SLAM IN starting up on \'{}\' port: {}'.format(*server_me_in))
socket_me_in.bind(server_me_in)
"""


############################################################
# FORWARDING MESSAGES
############################################################
print('Starting perc_out_slam_in...')
perc_out_slam_in = threading.Thread(target=tcplib.forward, args=(socket_perception_out, socket_slam_in))
perc_out_slam_in.start()

"""
ADD REMAINING CONNECTION THREADS
"""

perc_out_slam_in.join()

"""
ADD REMAINING THREADS JOIN
"""

print("Done.")