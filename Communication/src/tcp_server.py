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

IP_PORT_SENSORS_OUT_IMU =    10010
IP_PORT_SENSORS_OUT_SPEED =  10011
IP_PORT_CV_OUT1 =            10012

IP_PORT_SLAM_IN_CONES =      10020
IP_PORT_SLAM_IN_ME =         10021
IP_PORT_SLAM_OUT =           10022

IP_PORT_ME_IN_IMU =          10030
IP_PORT_ME_IN_SPEED =        10031
IP_PORT_ME_OUT_SLAM =        10032
IP_PORT_ME_OUT_CTRL =        10033

IP_PORT_PP_IN1 =             10040
IP_PORT_PP_OUT1 =            10041

IP_PORT_CTRL_IN_PP =         10050
IP_PORT_CTRL_IN_ME =         10051


############################################################
# SENSORS SOCKET(S)
############################################################
socket_sensors_imu = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_sensors_imu = (IP_ADDR, IP_PORT_SENSORS_OUT_IMU)
print('SENSORS OUT IMU starting up on \'{}\' port: {}'.format(*server_sensors_imu))
socket_sensors_imu.bind(server_sensors_imu)

socket_sensors_speed = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_sensors_speed = (IP_ADDR, IP_PORT_SENSORS_OUT_SPEED)
print('SENSORS OUT SPEED starting up on \'{}\' port: {}'.format(*server_sensors_speed))
socket_sensors_speed.bind(server_sensors_speed)

socket_cv_out = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_cv_out = (IP_ADDR, IP_PORT_CV_OUT1)
print('SENSORS OUT CONES starting up on \'{}\' port: {}'.format(*server_cv_out))
socket_cv_out.bind(server_cv_out)


"""
############################################################
# PERCEPTION SOCKET(S)
############################################################
socket_cv_out = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_cv_out = (IP_ADDR, IP_PORT_CV_OUT1)
print('Perception Out starting up on \'{}\' port: {}'.format(*server_cv_out))
socket_cv_out.bind(server_cv_out)
"""

############################################################
# SLAM SOCKET(S)
############################################################
socket_slam_in_cones = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_slam_in1 = (IP_ADDR, IP_PORT_SLAM_IN_CONES)
print('SLAM IN1 starting up on \'{}\' port: {}'.format(*server_slam_in1))
socket_slam_in_cones.bind(server_slam_in1)

socket_slam_in_me = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_slam_in2 = (IP_ADDR, IP_PORT_SLAM_IN_ME)
print('SLAM IN2 starting up on \'{}\' port: {}'.format(*server_slam_in2))
socket_slam_in_me.bind(server_slam_in2)

socket_slam_out1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_slam_out1 = (IP_ADDR, IP_PORT_SLAM_OUT)
print('SLAM OUT1 starting up on \'{}\' port: {}'.format(*server_slam_out1))
socket_slam_out1.bind(server_slam_out1)


############################################################
# MOTION ESTIMATION SOCKET(S)
############################################################
socket_me_in_speed = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_me_in_speed = (IP_ADDR, IP_PORT_ME_IN_SPEED)
print('MOTION ESTIMATION IN SPEED starting up on \'{}\' port: {}'.format(*server_me_in_speed))
socket_me_in_speed.bind(server_me_in_speed)

socket_me_in_imu = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_me_in_imu = (IP_ADDR, IP_PORT_ME_IN_IMU)
print('MOTION ESTIMATION IN IMU starting up on \'{}\' port: {}'.format(*server_me_in_imu))
socket_me_in_imu.bind(server_me_in_imu)


socket_me_out_slam = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_me_out1 = (IP_ADDR, IP_PORT_ME_OUT_SLAM)
print('MOTION ESTIMATION OUT1 starting up on \'{}\' port: {}'.format(*server_me_out1))
socket_me_out_slam.bind(server_me_out1)

socket_me_out_ctrl = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_me_out2 = (IP_ADDR, IP_PORT_ME_OUT_CTRL)
print('MOTION ESTIMATION OUT2 starting up on \'{}\' port: {}'.format(*server_me_out2))
socket_me_out_ctrl.bind(server_me_out2)


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
socket_ctrl_in_pp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_ctrl_in1 = (IP_ADDR, IP_PORT_CTRL_IN_PP)
print('CONTROL IN1 starting up on \'{}\' port: {}'.format(*server_ctrl_in1))
socket_ctrl_in_pp.bind(server_ctrl_in1)

socket_ctrl_in_me = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_ctrl_in2 = (IP_ADDR, IP_PORT_CTRL_IN_ME)
print('CONTROL IN2 starting up on \'{}\' port: {}'.format(*server_ctrl_in2))
socket_ctrl_in_me.bind(server_ctrl_in2)


############################################################
# FORWARDING MESSAGES
############################################################
imu_out_me_in = threading.Thread(target=tcplib.forward, args=('imu_out_me_in', socket_sensors_imu, socket_me_in_imu, True))
imu_out_me_in.start()

speed_out_me_in = threading.Thread(target=tcplib.forward, args=('speed_out_me_in', socket_sensors_speed, socket_me_in_speed, True))
speed_out_me_in.start()

cv_out_slam_in = threading.Thread(target=tcplib.forward, args=('cv_out_slam_in', socket_cv_out, socket_slam_in_cones, True))
cv_out_slam_in.start()

me_out_slam_in = threading.Thread(target=tcplib.forward, args=('me_out_slam_in', socket_me_out_slam, socket_slam_in_me, True))
me_out_slam_in.start()

slam_out_pp_in = threading.Thread(target=tcplib.forward, args=('slam_out_pp_in', socket_slam_out1, socket_pp_in1, True))
slam_out_pp_in.start()

pp_out_ctrl_in = threading.Thread(target=tcplib.forward, args=('pp_out_ctrl_in', socket_pp_out1, socket_ctrl_in_pp, True))
pp_out_ctrl_in.start()

me_out_ctrl_in = threading.Thread(target=tcplib.forward, args=('me_out_ctrl_in', socket_me_out_ctrl, socket_ctrl_in_me, True))
me_out_ctrl_in.start()

cv_out_slam_in.join()
me_out_slam_in.join()
slam_out_pp_in.join()
pp_out_ctrl_in.join()
me_out_ctrl_in.join()

print("Server closing properly.")