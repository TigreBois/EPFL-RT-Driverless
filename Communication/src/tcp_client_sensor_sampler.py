import socket
import sys
import pickle
import time
import tcplib

IP_PORT_OUT_IMU =   10010
IP_PORT_OUT_SPEED = 10011
IP_PORT_OUT_CONES = 10012
IP_ADDR = 'localhost'
MY_MODULE_NAME = 'SENSOR SAMPLER' # Please enter here an arbitrary name for your code, which will help in logging

datasource = 'circle_track_sensors_data.pickle'

print('INFO:', MY_MODULE_NAME, 'starting.')

######################################################
# INITIALIZE YOUR MODULE HERE
######################################################

# Create a TCP/IP socket for the input
sock_output_cones = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_output_cones = (IP_ADDR, IP_PORT_OUT_CONES)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for input.'.format(*server_output_cones))
#sock_output_cones.connect(server_output_cones)

# Create a TCP/IP socket for the output
sock_output_imu = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_output_imu = (IP_ADDR, IP_PORT_OUT_IMU)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for output.'.format(*server_output_imu))
#sock_output_imu.connect(server_output_imu)

# Create a TCP/IP socket for the output
sock_output_speed = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_output_speed = (IP_ADDR, IP_PORT_OUT_SPEED)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for output.'.format(*server_output_speed))
#sock_output_speed.connect(server_output_speed)

module_running = True

def parse_sensors(sensor_frame):
    # Layout: (x, y, speed, /acceleration,/ yaw, yaw_rate, [(color, x_rel, y_rel), (color, x_rel, y_rel), ...])

    sensor_data = {}

    sensor_data['x'] = sensor_frame[0]
    sensor_data['y'] = sensor_frame[1]
    sensor_data['speed'] = sensor_frame[2]

    #sensor_data['acceleration'] = sensor_frame[3]
    #sensor_data['yaw'] = sensor_frame[4]
    #sensor_data['yaw-rate'] = sensor_frame[5]
    #sensor_data['cones'] = sensor_frame[6]

    sensor_data['yaw'] = sensor_frame[3]
    sensor_data['yaw-rate'] = sensor_frame[4]
    sensor_data['cones'] = sensor_frame[5]

    return sensor_data

try:
    with open(datasource, 'rb') as picklefile:
        sensor_data = pickle.load(picklefile)

        for i in range(len(sensor_data)):

            sensors_frame = parse_sensors(sensor_data[i])

            cones_frame = [(sensors_frame['cones'][i][1], sensors_frame['cones'][i][2], 0.5, sensors_frame['cones'][i][0]) for i in range(len(sensors_frame['cones']))]


            imu_frame = (sensors_frame['x'], sensors_frame['y'], \
                            sensors_frame['speed'], \
                            # sensors_frame['acceleration'], \
                            sensors_frame['yaw'], sensors_frame['yaw-rate'])


            speed_frame = sensors_frame['speed']

            print('speed_frame', speed_frame)
            print('imu_frame', imu_frame)
            print('cones_frame', cones_frame)
            print("####################")

            """
            tcplib.sendData(sock_output_cones, cones_frame)
            tcplib.sendData(sock_output_imu, imu_frame)
            tcplib.sendData(sock_output_speed, speed_frame)
            """

            time.sleep(1/200)

finally:
    sock_input.close()
    sock_output.close()