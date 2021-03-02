"""lidar_controller controller."""

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, Lidar, LidarPoint 
from controller import GPS, Gyro, InertialUnit, Accelerometer # Motion estimation
import pickle
import pandas as pd
import time
from math import cos, pi
from cones import Cones
from coordstransformslocal import webots2ISO, world2car

# SENSORS_DATA_SAVING_TIME_INTERVAL = 0.1 # in seconds
SENSORS_DATA_FILENAME = "sensors_data"

CONE_SCALING = 0.5 # How cones were scaled when created (Will be used to ease search of cones in Webots window)
CONE_PERCEPTION_DISTANCE = 20 # in meters
DO_PERCEPTION = False
BASE_SPEED = 2.0


# We will store sensors data here
sensors_data = []

# Get cone coordinates
cones = Cones("../../../cone_coordinates.csv")

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# motors = []
# wheels_names = ["wheel1", "wheel2", "wheel3", "wheel4"]
# for name in wheels_names:
    # motor = robot.getMotor(name)
    # motor.setPosition(float('inf'))
    # motor.setVelocity(0.0)
    # motors.append(motor)

# Activate sensors
lidar = robot.getDevice("lidar")
lidar.enable(timestep) # Are you sure?
lidar.enablePointCloud()

gps = robot.getDevice("gps")
gps.enable(timestep)

gyro = robot.getDevice("gyro")
gyro.enable(timestep)
# gyro.xAxis, gyro.zAxis = False, False # We need only yaw rate

imu = robot.getDevice("imu")
imu.enable(timestep)
# imu.xAxis, imu.zAxis = False, False # We need only yaw angle

accelerometer = robot.getDevice("accelerometer")
accelerometer.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
step = 0
while robot.step(timestep) != -1:
    print("\n\n\nNew step")
    if DO_PERCEPTION:
        raise NotImplementedError
        # Read the sensors:
        range_image = lidar.getRangeImageArray()
        point_cloud = lidar.getPointCloud()
    
        point_cloud_to_save = []
        for point in point_cloud:
            point_cloud_to_save.append([point.x, point.y, point.z,
                                        point.layer_id, point.time])
        # Process sensor data here.
        with open(f"lidar_range_image_{step}.pickle", "wb") as f:
            pickle.dump(range_image, f)
        with open(f"lidar_point_cloud_{step}.pickle", "wb") as f:
            pickle.dump(point_cloud_to_save, f)
       
    # Work with sensors 
    robot_coord = gps.getValues()
    print("Robot coordinates:", robot_coord)
    
    robot_speed = gps.getSpeed()
    print("Robot speed", robot_speed)
    
    robot_orientation = imu.getRollPitchYaw() # Need [2] - index of yaw
    yaw = robot_orientation[2]
    print("Yaw angle is", yaw)
    
    acceleration = accelerometer.getValues()
    # Need to check calculations:
    acceleration_dir = acceleration[0] * cos(yaw) + acceleration[2] * cos(yaw + pi/2)
    print("Acceleration is", acceleration)
    
    angle_velocity = gyro.getValues() # Need [1] - index of yaw
    yaw_rate = angle_velocity[1]
    print("Yaw rate is", yaw_rate)
    
    
    robot_coord_ISO = webots2ISO(robot_coord)
    print("Robot coords ISO:", robot_coord_ISO)
    # yaw_ISO = yaw + pi/2 # For previous version of car (New version is rotated)
    yaw_ISO = yaw + pi/2
    print("yaw ISO:", yaw_ISO)
    robot_orientation_ISO = (0, 0, yaw_ISO)
    cones_car_ISO = cones.get_cones_car_ISO_coords(robot_coord_ISO, robot_orientation_ISO)
    cones_car_ISO = [(cone[0], cone[1], cone[2], cone[3],
                      cone[4] / CONE_SCALING, cone[5] / CONE_SCALING) for cone in cones_car_ISO]
    print("Cones in car ISO coordinates:")
    for cone in cones_car_ISO:
        print(cone)
        
    cones_within_dist = [cone for cone in cones_car_ISO
        if 0 < cone[1] < CONE_PERCEPTION_DISTANCE and -CONE_PERCEPTION_DISTANCE < cone[2] < CONE_PERCEPTION_DISTANCE]
    print("Visible cones in car ISO coordinates")
    for cone in cones_within_dist:
        print(cone)
        
    sensors_data_new = [robot_coord_ISO[0], 
                        robot_coord_ISO[1], 
                        robot_speed,
                        acceleration_dir ,
                        yaw_ISO, yaw_rate, 
                        [(cone[0], cone[1], cone[2]) for cone in cones_within_dist]]
                                                
    sensors_data.append(tuple(sensors_data_new))
    
    # Enter here functions to send actuator commands, like:
    # for i, motor in enumerate(motors):
        # coef = 1 if i % 2 == 0 else 0.8
        # motor.setVelocity(coef * BASE_SPEED)
        
    step += 1
    
# Enter here exit cleanup code.

# Save sensors data as .csv and .pickle file
print("Saving results")
with open(SENSORS_DATA_FILENAME + ".pickle", "wb") as f: # save list of tuples as .pickle
    pickle.dump(sensors_data, f)
    

# TODO do we need to disable lidar, gps, robot here?