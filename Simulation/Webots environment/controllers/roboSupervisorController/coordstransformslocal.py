import numpy as np

def webots2ISO(coordinates_iso):
    # Webots coordinate system is:
    # x left
    # y up
    # z forward

    # Returns the coordinates in ISO 8855 frame

    return np.array([coordinates_iso[2], coordinates_iso[0], coordinates_iso[1]])

def ISO2webots(coordinates_webots):
    # Webots coordinate system is:
    # x left
    # y up
    # z forward

    # Returns the coordinates in Webots frame

    return np.array([coordinates_webots[1], coordinates_webots[2], coordinates_webots[0]])


def world2car(pos_w, car_pos_w, car_orientation_w):
    # car_w = [x, y, z, heading] coordinates of car in world frame
    # pos_w = [x, y, z], coordinates of an arbitrary object in world frame
    # Returns: pos_c; coordinates of pos_w in car frame.

    # We assume car is always flat on the flat ground.

    # NB: We ignore roll and pitch

    pos_w = np.concatenate((np.reshape(np.array(pos_w), (3,1)), np.array([[1]])))

    roll = car_orientation_w[0]
    pitch = car_orientation_w[1]
    yaw = car_orientation_w[2]

    # Inverse rotation of the car
    R_z_yaw_inv = np.array(
                    [[np.cos(-yaw), -np.sin(-yaw),  0], 
                    [np.sin(-yaw),  np.cos(-yaw),   0], 
                    [0,             0,              1]]
                )

    # Inverse translation of the car
    T_v_inv = np.array(
                    [[1,    0,  0,  -car_pos_w[0]], 
                    [0,     1,  0,  -car_pos_w[1]], 
                    [0,     0,  1,  -car_pos_w[2]], 
                    [0,     0,  0,      1]]
                )

    translated = np.matmul(T_v_inv, pos_w)[:3]

    return np.matmul(R_z_yaw_inv, translated)