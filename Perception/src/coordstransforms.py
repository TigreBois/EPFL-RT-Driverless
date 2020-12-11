import numpy as np

# This follows the Mathworks proposal: https://www.mathworks.com/help/driving/ug/coordinate-systems.html

# All angles are in radians
# All distances are in meters

##########################################################
# WORLD COORDINATES DESCRIPTION:
##########################################################
# The origin of the world coordinate system is at:
# The x axis is oriented towards: N/A (Only matters if the position in the real world matters)
# The y axis is oriented towards: N/A (Only matters if the position in the real world matters)
# The z axis is oriented towrads: N/A (Only matters if the position in the real world matters)
# NB: The world and vehicle coordinates are set such that at initialization, the vehicle
# is located at [0, 0, 0], with 0° heading.

##########################################################
# VEHICLE COORDINATES DESCRIPTION:
##########################################################
# The origin of the vehicle coordinate system is at:
# The x axis is oriented towards: Forward
# The y axis is oriented towards: Left
# The z axis is oriented towrads: Up

##########################################################
# SENSOR COORDINATES DESCRIPTION:
##########################################################
# The origin of each sensor coordinate system is at:
# The x axis is oriented towards: Forward
# The y axis is oriented towards: Left
# The z axis is oriented towrads: Up

##########################################################
# CAMERA PIXEL COORDINATES
##########################################################
# Specific to the camera, we need the focal distance f
# The pixel x axis is oriented towards: Right
# The pixel y axis is oriented towards: Down


def world2car():



def car2sensor():



def cam2pix():



def pix2cam():



def sensor2car():



def car2world(pos_v, vehicle_transform):
    # We assume vehicle is always flat on the flat ground.
    
    pos_v = np.array(pos_v).view((3,1))
    teta = vehicle_transform.heading
    
    R_z_teta = np.array([[np.cos(teta), -np.sin(teta), 0], [np.sin(teta), np.cos(teta), 0], [0, 0, 1]])
    T_v = np.array([[1, 0, 0, vehicle_transform.x], [0, 1, 0, vehicle_transform.y], [0, 0, 0, vehicle_transform.z], [0, 0, 0, 1]])

    rotated = np.ones((4,1))
    rotated[:3,:] = np.matmul(R_z_teta, pos_v)

    return np.matmul(T_v, rotated)[:3]
