import socket
import sys
import pickle
import time
import tcplib

import numpy as np
import math

def kalman_update(x, data, P, Q, R, I, dt):
    
    #x0 is x, x1 is y, x2 is heading, x3 is velocity, x4 is acceleration, x5 is yaw rate
    
    
    ux = (1/x[5]**2) * \
 ((x[3]*x[5] + x[4]*x[5]*dt)* \
  np.sin(x[2] + x[5]*dt) + x[4]*np.cos(x[2] + x[5]*dt) - x[3]*x[5]*np.sin(x[2]) - np.cos(x[2])*x[4])
    uy = (1/x[5]**2) * \
 ((-x[3]*x[5] - x[4]*x[5]*dt)* np.cos(x[2] + x[5]*dt) \
 + x[4]*np.sin(x[2] + x[5]*dt) + x[3]*x[5]*np.cos(x[2]) - x[4]*np.sin(x[2]))


    x[0] = x[0] + ux
    x[1] = x[1] + uy
    x[2] = x[2] + dt*x[5]
    x[3] = x[3] + dt*x[4]
    x[4] = x[4]
    x[5] = x[5]
    
    # Calculate the Jacobian of the Dynamic Matrix A
    # see "Calculate the Jacobian of the Dynamic Matrix with respect to the state vector"
    a13 = float((1/x[5]**2) * (x[4]*np.sin(x[2])- x[4]*np.sin(dt*x[5]+x[2]) - x[3]*x[5]*np.cos(x[2]) + (dt*x[4]*x[5] + x[3]*x[5])*np.cos(dt*x[5]+x[2]))) 
    a23 = float((1/x[5]**2) * (-x[4]*np.cos(x[2])+ x[4]*np.cos(dt*x[5]+x[2]) - x[3]*x[5]*np.sin(x[2]) - (-dt*x[4]*x[5] - x[3]*x[5])*np.sin(dt*x[5]+x[2]))) 
    
    a14 = float((1/x[5]**2) * (-x[5]*np.sin(x[2]) + x[5]*np.sin(dt*x[5]+x[2])))
    a24 = float((1/x[5]**2) * (x[5]*np.cos(x[2]) - x[5]*np.cos(dt*x[5]+x[2])))
    
    a15 = float((1/x[5]**2) * (dt*x[5]*np.sin(dt*x[5]+x[2])-np.cos(x[2])+np.cos(dt*x[5] + x[2])))
    a25 = float((1/x[5]**2) * (-dt*x[5]*np.cos(dt*x[5]+x[2])-np.sin(x[2])+np.sin(dt*x[5] + x[2])))
    
    a161 = float((1/x[5]**2) * (-dt*x[4]*np.sin(dt*x[5]+x[2])+dt*(dt*x[4]*x[5]+x[3]*x[5])*np.cos(dt*x[5]+x[2])-x[3]*np.sin(x[2])+(dt*x[4]+x[3])*np.sin(dt*x[5]+x[2])))
    a162 = float((1/x[5]**3) * (-x[4]*np.cos(x[2])+x[4]*np.cos(dt*x[5]+x[2])-x[3]*x[5]*np.sin(x[2])+(dt*x[4]*x[5]+x[3]*x[5])*np.sin(dt*x[5]+x[2])))
    a16 = a161-2*a162
    
    a261 = float((1/x[5]**2) * (dt*x[4]*np.cos(dt*x[5]+x[2])-dt*(-dt*x[4]*x[5]-[3]*x[5])*np.sin(dt*x[5]+x[2])+x[3]*np.cos(x[2])+(-dt*x[4]-x[3])*np.cos(dt*x[5]+x[2])))
    a262 = float((1/x[5]**3) * (-x[4]*np.sin(x[2])+x[4]*np.sin(dt*x[5]+x[2])+x[3]*x[5]*np.cos(x[2])+(-dt*x[4]*x[5]-x[3]*x[5])*np.cos(dt*x[5]+x[2])))
    a26 = a161-2*a162


    JA = np.matrix([[1.0, 0.0, a13, a14, a15, a16],
                    [0.0, 1.0, a23, a24, a25, a26],
                    [0.0, 0.0, 1.0, 0.0, 0.0, dt],
                    [0.0, 0.0, 0.0, 1.0, dt, 0.0], 
                    [0.0, 0.0, 0.0, 0.0, 1.0, 0.0], 
                    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
    
    
    # Project the error covariance ahead
    P = JA*P*JA.T + Q
    
    # Measurement Update (Correction)
    # ===============================
    # Measurement Function
    hx = np.matrix([[float(x[0])],
                    [float(x[1])],
                    [float(x[2])],
                    [float(x[3])],
                    [float(x[4])],
                    [float(x[5])]])


    JH = np.diag([1.0,1.0,1.0,1.0,1.0,1.0])      
    S = JH*P*JH.T + R
    K = (P*JH.T) * np.linalg.inv(S)

    # Update the estimate via
    #Z = measurements[:,filterstep].reshape(JH.shape[0],1)
    #Z = data.reshape(JH.shape[0],1)
    Z = data
    y = Z - (hx)
    x = x + (K*y)

    # Update the error covariance
    P = (I - (K*JH))*P
    
    return (x, P)



IP_PORT_IN_IMU =  10030
IP_PORT_IN_SPEED = 10031
IP_PORT_OUT_SLAM =  10032
IP_PORT_OUT_CONTROL = 10033
IP_ADDR = 'localhost'
MY_MODULE_NAME = 'motion_estimation'

print('INFO:', MY_MODULE_NAME, 'starting.')


# Init :
numstates = 6
dt = 1.0/200.0

varGPS = 6.0 
varspeed = 1.0 
varacc = 1.0
varyaw = 0.1 # Variances on sensor data I had. To be changed for the car when we have the sensors
R = np.diag([varGPS**2, varGPS**2, varyaw**2, varspeed**2,varacc**2, varspeed**2])
I = np.eye(numstates)

P = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
sPos     = 0.5*8.8*dt**2  # assume 8.8m/s2 as maximum acceleration, forcing the vehicle
sCourse  = 0.1*dt # assume 0.1rad/s as maximum turn rate for the vehicle
sVelocity= 8.8*dt # assume 8.8m/s2 as maximum acceleration, forcing the vehicle. Again, to be changed
Q = np.diag([sPos**2, sPos**2, sCourse**2, sVelocity**2, sCourse**2, sVelocity**2])


# mx, my, course, speed, acceleration, yaw, GPS = getValuesSensor()
# x = np.matrix([[mx[0], my[0], course[0]/180.0*np.pi, speed[0]/3.6+0.001, acceleration[0]/3.6+0.0001 , yaw[0]/180.0*np.pi]]).T
# x = np.matrix([[0, 0, 0, 0, 0 , 0]]).T
# measurements = np.vstack((mx, my, course/180.0*np.pi, speed/3.6, acceleration/3.6, yaw/180.0*np.pi))
# i = 0

# Create a TCP/IP socket for the input
socket_imu = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address_in_imu = (IP_ADDR, IP_PORT_IN_IMU)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for input.'.format(*server_address_in_imu))
socket_imu.connect(server_address_in_imu)

socket_speed = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address_in_speed = (IP_ADDR, IP_PORT_IN_SPEED)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for input.'.format(*server_address_in_speed))
socket_speed.connect(server_address_in_speed)


# Create a TCP/IP socket for the output
socket_slam = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address_out_slam = (IP_ADDR, IP_PORT_OUT_SLAM)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for output.'.format(*server_address_out_slam))
socket_slam.connect(server_address_out_slam)

socket_control = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address_out_control = (IP_ADDR, IP_PORT_OUT_CONTROL)
print('INFO: \'', MY_MODULE_NAME, '\' connecting to {} port {} for output.'.format(*server_address_out_control))
socket_control.connect(server_address_out_control)


module_running = True
first_mes = True

try:
    while module_running:
        

        speed = tcplib.receiveData(socket_speed)
        imu_frame = tcplib.receiveData(socket_imu)
        
        measurements = np.matrix([[imu_frame[0], imu_frame[1], imu_frame[4], speed, imu_frame[3], imu_frame[5]]]).T
        if first_mes:
            x = measurements
            first_mes = False
        
        #i+=1
        #x, P = kalman_update(x, measurements[:,i], P, Q, R, I, GPS)
        x, P = kalman_update(x, measurements, P, Q, R, I, dt)
        
        to_send = x.A1
        
        tcplib.sendData(socket_control, to_send)
        tcplib.sendData(socket_slam, to_send)


        if false:
            module_running = False

finally:
    sock_input.close()
    sock_output.close()
    

