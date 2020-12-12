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

IP_PORT_CV_OUT1 =            10010

IP_PORT_SLAM_IN1 =           10020
IP_PORT_SLAM_IN2 =           10021
IP_PORT_SLAM_OUT1 =          10022

IP_PORT_ME_OUT1 =            10030
IP_PORT_ME_OUT2 =            10031

IP_PORT_PP_IN1 =             10040
IP_PORT_PP_OUT1 =            10041

IP_PORT_CTRL_IN1 =           10050
IP_PORT_CTRL_IN2 =           10051


############################################################
# PERCEPTION SOCKET(S)
############################################################
socket_cv_out = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_cv_out = (IP_ADDR, IP_PORT_CV_OUT1)
print('Perception Out starting up on \'{}\' port: {}'.format(*server_cv_out))
socket_cv_out.bind(server_cv_out)


############################################################
# SLAM SOCKET(S)
############################################################
socket_slam_in1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_slam_in1 = (IP_ADDR, IP_PORT_SLAM_IN1)
print('SLAM IN1 starting up on \'{}\' port: {}'.format(*server_slam_in1))
socket_slam_in1.bind(server_slam_in1)

socket_slam_in2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_slam_in2 = (IP_ADDR, IP_PORT_SLAM_IN2)
print('SLAM IN2 starting up on \'{}\' port: {}'.format(*server_slam_in2))
socket_slam_in2.bind(server_slam_in2)

socket_slam_out1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_slam_out1 = (IP_ADDR, IP_PORT_SLAM_OUT1)
print('SLAM OUT1 starting up on \'{}\' port: {}'.format(*server_slam_out1))
socket_slam_out1.bind(server_slam_out1)


############################################################
# MOTION ESTIMATION SOCKET(S)
############################################################
socket_me_out1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_me_out1 = (IP_ADDR, IP_PORT_ME_OUT1)
print('MOTION ESTIMATION OUT1 starting up on \'{}\' port: {}'.format(*server_me_out1))
socket_me_out1.bind(server_me_out1)

socket_me_out2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_me_out2 = (IP_ADDR, IP_PORT_ME_OUT2)
print('MOTION ESTIMATION OUT2 starting up on \'{}\' port: {}'.format(*server_me_out2))
socket_me_out2.bind(server_me_out2)


############################################################
# PATH PLANNING SOCKET(S)
############################################################
socket_pp_in1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_pp_in1 = (IP_ADDR, IP_PORT_PP_IN1)
print('PATH PLANNING IN1 starting up on \'{}\' port: {}'.format(*server_pp_in1))
socket_pp_in1.bind(server_pp_in1)

socket_pp_out1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_pp_out1 = (IP_ADDR, IP_PORT_PP_OUT1)
print('PATH PLANNING OUT1 starting up on \'{}\' port: {}'.format(*server_pp_out1))
socket_pp_out1.bind(server_pp_out1)


############################################################
# CONTROL SOCKET(S)
############################################################
socket_ctrl_in1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_ctrl_in1 = (IP_ADDR, IP_PORT_CTRL_IN1)
print('CONTROL IN1 starting up on \'{}\' port: {}'.format(*server_ctrl_in1))
socket_ctrl_in1.bind(server_ctrl_in1)

socket_ctrl_in2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_ctrl_in2 = (IP_ADDR, IP_PORT_CTRL_IN2)
print('CONTROL IN2 starting up on \'{}\' port: {}'.format(*server_ctrl_in2))
socket_ctrl_in2.bind(server_ctrl_in2)


############################################################
# FORWARDING MESSAGES
############################################################
cv_out_slam_in = threading.Thread(target=tcplib.forward, args=('cv_out_slam_in', socket_cv_out, socket_slam_in1, True))
cv_out_slam_in.start()

me_out_slam_in = threading.Thread(target=tcplib.forward, args=('me_out_slam_in', socket_me_out1, socket_slam_in2, True))
me_out_slam_in.start()

slam_out_pp_in = threading.Thread(target=tcplib.forward, args=('slam_out_pp_in', socket_slam_out1, socket_pp_in1, True))
slam_out_pp_in.start()

pp_out_ctrl_in = threading.Thread(target=tcplib.forward, args=('pp_out_ctrl_in', socket_pp_out1, socket_ctrl_in1, True))
pp_out_ctrl_in.start()

me_out_ctrl_in = threading.Thread(target=tcplib.forward, args=('me_out_ctrl_in', socket_me_out2, socket_ctrl_in2, True))
me_out_ctrl_in.start()

cv_out_slam_in.join()
me_out_slam_in.join()
slam_out_pp_in.join()
pp_out_ctrl_in.join()
me_out_ctrl_in.join()

print("Server closing properly.")