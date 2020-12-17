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

# Beginning of my code

max_iter = 10  # Number of middle points to be selected
point_step = 0.5  # Space between points in output in meters
origin = np.array([0, 0])  # origin = car position
rad_to_deg = 180 / pi


# Sort list a of points according to their norm
def sortNorm(a): return a[np.argsort(np.linalg.norm(a, axis=1))]


# Compute the angle between vector v and w
def theta(v, w): return arccos(v.dot(w) / (norm(v) * norm(w)))


# Compute list of angles corresponding to the list of points ps
def getAngles(ps):
    v = np.subtract(ps[1:], ps[:-1])
    allAngles = [np.arctan2(v[0][0], v[0][1])]
    for i in range(1, np.ma.size(v, 0)):
        allAngles.append(theta(v[i - 1], v[i]))
    return np.array(allAngles) * rad_to_deg


# Return the middle points of a triangle
def midTriangle(a): return [a[0] + a[1], a[1] + a[2], a[2] + a[0]]


# Compute the middle path between the cones
def middle_path(points):
    # Compute the Delaunay Triangulation formed by the cones
    tri = Delaunay(points)

    # Compute the middle of all the edges of the triangulation
    midPoints = (np.apply_along_axis(midTriangle, 1, points[tri.simplices]) * 0.5).reshape(-1, 2)

    norms = np.linalg.norm(midPoints, axis=1)
    closest = midPoints[np.argmin(norms)]

    # Find midPoints that are in 2 different triangles
    u, indices = np.unique(midPoints, return_index=True, axis=0)
    e = np.delete(midPoints, indices, axis=0)
    d = sortNorm(e)

    # Add origin and first point to which the car should go
    d = np.insert(d, 0, closest, axis=0)
    d = np.insert(d, 0, origin, axis=0)

    # From the remaining points, filter points creating big angle turns
    filtered = np.array(d)
    angles = getAngles(filtered)
    i = 0
    while i < angles.size:
        while (angles[i] < 30) & (i < angles.size - 1) & (i < max_iter):
            i = i + 1
        filtered = np.delete(filtered, i + 1, 0)
        angles = getAngles(filtered)

    # Make a path with sufficient number of points with interpolation
    f = interp1d(filtered[:, 0], filtered[:, 1])
    x_new = np.arange(0, filtered[-1][0], point_step)
    y_new = f(x_new)
    output = np.vstack((x_new, y_new)).T

    return output


# Beginning of Wrapper

IP_PORT_IN = 10040                  # input form SLAM
IP_PORT_OUT = 10041                 # output to control
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

        path = middle_path(msg)

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
