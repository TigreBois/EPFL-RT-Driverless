import numpy as np

# This follows the Mathworks proposal: https://www.mathworks.com/help/driving/ug/coordinate-systems.html

# All angles are in radians
# All distances are in meters (including focal distance)
# Roll, pitch, yaw are respectively direct rotations around x,y, z

##########################################################
# WORLD COORDINATES DESCRIPTION:
##########################################################
# The origin of the world coordinate system is at:
# The x axis is oriented towards: N/A (Only matters if the position in the real world matters)
# The y axis is oriented towards: N/A (Only matters if the position in the real world matters)
# The z axis is oriented towrads: N/A (Only matters if the position in the real world matters)
# NB: The world and car coordinates are set such that at initialization, the car
# is located at [0, 0, 0], with 0° heading.

##########################################################
# CAR COORDINATES DESCRIPTION:
##########################################################
# The origin of the car coordinate system is at:
# The x axis is oriented towards: Forward
# The y axis is oriented towards: Left
# The z axis is oriented towrads: Up

##########################################################
# SENSOR COORDINATES DESCRIPTION:
##########################################################
# The origin of each sensor coordinate system is at: the sensor
# The x axis is oriented towards: Forward
# The y axis is oriented towards: Left
# The z axis is oriented towrads: Up

##########################################################
# CAMERA PIXEL COORDINATES
##########################################################
# Specific to the camera, we need the focal distance f (in [m])
# The origin of the pixel frame is the top left corner
# The pixel x axis is oriented towards: Right
# The pixel y axis is oriented towards: Down

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


def car2sensor(pos_c, sensor_pos_c, sensor_orientation_c):
    # sensor_pos_c = [x, y, z] position of sensor in car frame
    # sensor_orientation_c = [roll, pitch, yaw] orientation of sensor in car frame
    # pos_c = [x, y, z], coordinates of an arbitrary object in car frame
    # Returns: pos_s; coordinates of pos_c in sensor frame.

    # NB: We ignore sensor roll and yaw
    
    pos_c = np.concatenate((np.reshape(np.array(pos_c), (3,1)), np.array([[1]])))
    
    roll = sensor_orientation_c[0]
    pitch = sensor_orientation_c[1]
    yaw = sensor_orientation_c[2]
    
    # Inverse rotation of the sensor
    R_y_pitch_inv = np.array(
                            [[np.cos(-pitch),   0,      np.sin(-pitch)], 
                            [0,                 1,      0               ], 
                            [-np.sin(-pitch),   0,      np.cos(-pitch)]]
                        )

    # Inverse translation of the sensor
    T_s_inv = np.array(
                        [[1,    0,  0,  -sensor_pos_c[0]], 
                        [0,     1,  0,  -sensor_pos_c[1]], 
                        [0,     0,  1,  -sensor_pos_c[2]], 
                        [0,     0,  0,      1]]
                    )

    translated = np.matmul(T_s_inv, pos_c)[:3]

    return np.matmul(R_y_pitch_inv, translated)


def cam2pix(pos_cam, focal_dist):
    pass
    """
    # focal distance in meters

    x_cam / y_cam = f / y_pix => y_pix = f * y_cam / x_cam
    x_cam / z_cam = f / z_pix => z_pix = f * z_cam / x_cam

    y_pix       -f/x_cam     y_cam
    z_pix                   z_cam
    """


def pix2cam():
    pass


def sensor2car(pos_s, sensor_pos_c, sensor_orientation_c):
    # sensor_pos_c = [x, y, z] position of sensor in car frame
    # sensor_orientation_c = [roll, pitch, yaw] orientation of sensor in car frame
    # pos_s = [x, y, z], coordinates of an arbitrary object in sensor frame
    # Returns: pos_c; coordinates of pos_s in car frame.

    # NB: We ignore sensor roll and yaw
    
    pos_s = np.reshape(np.array(pos_s), (3,1))
    
    roll = sensor_orientation_c[0]
    pitch = sensor_orientation_c[1]
    yaw = sensor_orientation_c[2]
    
    R_y_pitch = np.array(
                        [[np.cos(pitch),    0,  np.sin(pitch)], 
                        [0,                 1,      0], 
                        [-np.sin(pitch),    0,  np.cos(pitch)]]
                    )
    T_s = np.array(
                [[1,    0,  0,  sensor_pos_c[0]], 
                [0,     1,  0,  sensor_pos_c[1]], 
                [0,     0,  1,  sensor_pos_c[2]], 
                [0,     0,  0,      1]]
            )

    rotated = np.ones((4,1))
    rotated[:3,:] = np.matmul(R_y_pitch, pos_s)

    return np.matmul(T_s, rotated)[:3]
    


def car2world(pos_c, car_pos_w, car_orientation_w):
    # car_pos_w = [x, y, z] position of car in world frame
    # car_orientation_w = [roll, pitch, yaw] orientation of car in world frame (heading = yaw)
    # pos_c = [x, y, z], coordinates of an arbitrary object in car frame
    # Returns: pos_w; coordinates of pos_c in world frame.

    # NB: We ignore car roll and pitch
    
    pos_c = np.reshape(np.array(pos_c), (3,1))
    
    roll = car_orientation_w[0]
    pitch = car_orientation_w[1]
    yaw = car_orientation_w[2]
    
    R_z_yaw = np.array([
                    [np.cos(yaw),   -np.sin(yaw),   0], 
                    [np.sin(yaw),   np.cos(yaw),    0], 
                    [0,                 0,          1]
                ])
    T_v = np.array([
                [1,     0,  0,  car_pos_w[0]], 
                [0,     1,  0,  car_pos_w[1]], 
                [0,     0,  1,  car_pos_w[2]], 
                [0,     0,  0,      1]
            ])

    rotated = np.ones((4,1))
    rotated[:3,:] = np.matmul(R_z_yaw, pos_c)

    return np.matmul(T_v, rotated)[:3]

"""
# SMALL UNIT TEST
print("###########")
car_pos_w = np.array([3, 2, 0])
car_orient_w = np.array([0, 0, np.pi/2])
point_c = np.array([[1, 2, 0]])
print(point_c)
point_w = car2world(point_c, car_pos_w, car_orient_w)
print(np.transpose(point_w))
point_c = world2car(point_w, car_pos_w, car_orient_w)
print(np.transpose(point_c))

print("###########")
point_c = np.array([[5, 0, 1]])
sensor_pos_c = np.array([0, 0, 1])
sensor_orient_c = np.array([0, np.pi/8, 0])
point_s = car2sensor(point_c, sensor_pos_c, sensor_orient_c)
print(point_c)
print(np.transpose(point_s))
point_c = sensor2car(point_s, sensor_pos_c, sensor_orient_c)
print(np.transpose(point_c))

print("###########")
car_pos_w = np.array([3, 2, 0])
car_pos_webots = ISO2webots(car_pos_w)
print(car_pos_w)
print(car_pos_webots)
print(webots2ISO(car_pos_webots))
"""