"""lidar_controller controller."""

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, Lidar, LidarPoint 
from controller import GPS, Gyro, InertialUnit, Accelerometer # Motion estimation
from controller import PositionSensor
from vehicle import Driver
import numpy as np
import pickle
import pandas as pd
import time
from math import cos, pi
import shutil
import os

from cones import Cones
from coordstransformslocal import webots2ISO, world2car

def convert_radius_to_steering_angle(radius, wheelbase):
    return wheelbase / np.sqrt(radius ** 2 - wheelbase ** 2 / 4)
    
# SENSORS_DATA_SAVING_TIME_INTERVAL = 0.1 # in seconds
SENSORS_DATA_FILENAME = "sensors_data"
LIDAR_DATA_FOLDER = "../../../lidar_data"

CONE_SCALING = 0.5 # How cones were scaled when created (Will be used to ease search of cones in Webots window)
CONE_PERCEPTION_DISTANCE = 20 # in meters
DO_PERCEPTION = True
BASE_SPEED = 2.0

def init_sensors(robot, timestep):
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
    
    sensors = {"lidar": lidar, "gps": gps, "gyro": gyro, "imu": imu, "accelerometer": accelerometer}
    return sensors

def get_sensor_data(sensors):
    gps_values = sensors["gps"].getValues()
    print("Robot coordinates:", gps_values)
    
    speed = sensors["gps"].getSpeed()
    print("Robot speed", speed)
    
    roll_pitch_yaw = sensors["imu"].getRollPitchYaw() # Need [2] - index of yaw
    yaw = roll_pitch_yaw[2]
    print("Yaw angle is", roll_pitch_yaw, "=>", yaw)
    
    acceleration_values = sensors["accelerometer"].getValues()
    # Need to check calculations:
    acceleration = acceleration_values[0] * cos(yaw) + acceleration_values[2] * cos(yaw + pi/2)
    print("Acceleration is", acceleration_values, "=>", acceleration)
    
    angle_velocity = sensors["gyro"].getValues() # Need [1] - index of yaw
    yaw_rate = angle_velocity[1]
    print("Yaw rate is", angle_velocity, "=>", yaw_rate)
    
    sensor_data = {"robot_coord": gps_values, "robot_speed": speed, "yaw": yaw,
                    "acceleration": acceleration, "yaw_rate": yaw_rate}
    return sensor_data

# We will store sensors data here
sensors_data = []
if DO_PERCEPTION:
    # Delete previous lidar data
    if os.path.isdir(LIDAR_DATA_FOLDER):
        shutil.rmtree(LIDAR_DATA_FOLDER)
    os.mkdir(LIDAR_DATA_FOLDER)
    
# Get cone coordinates
cones = Cones("../../../cone_coordinates.csv")

# create the Robot instance.
# robot = Robot()
driver = Driver()

# get the time step of the current world.
# timestep = int(robot.getBasicTimeStep())
timestep = int(driver.getBasicTimeStep())

# Activate sensors
# sensors = init_sensors(robot, timestep)
sensors = init_sensors(driver, timestep)


wheelbase = 1.57 # [m]
tire_radius = 0.4604 # [m]
circle_radius = 5 # [m]
speed = 5 # [m/s]

angle = convert_radius_to_steering_angle(circle_radius, wheelbase)
print(angle)

# setting initial speed and steering angle
driver.setSteeringAngle(-angle)
driver.setCruisingSpeed(0)


step = 0
# while robot.step(timestep) != -1:
while driver.step() != -1:
    print("############################################################################")
   

    if DO_PERCEPTION:
        # Read the sensors:
        # range_image = sensors["lidar"].getRangeImageArray()
        point_cloud = sensors["lidar"].getPointCloud()
    
        point_cloud_to_save = []
        for point in point_cloud:
            point_cloud_to_save.append([point.x, point.y, point.z,
                                        point.layer_id, point.time])
        # Process sensor data here.
        # with open(f"{LIDAR_DATA_FOLDER}/lidar_range_image_{step}.pickle", "wb") as f:
            # pickle.dump(range_image, f)
        with open(f"{LIDAR_DATA_FOLDER}/lidar_point_cloud_{step}.pickle", "wb") as f:
            pickle.dump(point_cloud_to_save, f)

    # Work with sensors 
    sensor_data = get_sensor_data(sensors)

    
    # Manipulate sensor data
    robot_coord = sensor_data["robot_coord"]
    robot_speed = sensor_data["robot_speed"]
    yaw = sensor_data["yaw"]
    acceleration = sensor_data["acceleration"]
    yaw_rate = sensor_data["yaw_rate"]
    
    robot_coord_ISO = webots2ISO(robot_coord)
    print("Robot coords ISO:", robot_coord_ISO)
    # yaw_ISO = yaw + pi/2 # For previous version of car (New version is rotated)
    yaw_ISO = yaw + pi/2
    print("yaw ISO:", yaw_ISO)
    robot_orientation_ISO = (0, 0, yaw_ISO)
    cones_car_ISO = cones.get_cones_car_ISO_coords(robot_coord_ISO, robot_orientation_ISO)
    cones_car_ISO = [(cone[0], cone[1], cone[2], cone[3],
                      cone[4] / CONE_SCALING, cone[5] / CONE_SCALING) for cone in cones_car_ISO]
    # print("Cones in car ISO coordinates:")
    # for cone in cones_car_ISO:
        # print(cone)
        
    cones_within_dist = [cone for cone in cones_car_ISO
        if 0 < cone[1] < CONE_PERCEPTION_DISTANCE and -CONE_PERCEPTION_DISTANCE < cone[2] < CONE_PERCEPTION_DISTANCE]
    # print("Visible cones in car ISO coordinates")
    # for cone in cones_within_dist:
        # print(cone)
        
    sensors_data_new = [robot_coord_ISO[0], 
                        robot_coord_ISO[1], 
                        robot_speed,
                        acceleration,
                        yaw_ISO, yaw_rate, 
                        [(cone[0], cone[1], cone[2]) for cone in cones_within_dist]]
                                                
    sensors_data.append(tuple(sensors_data_new))

    # Control
    driver.setCruisingSpeed(speed)
    driver.setSteeringAngle(0)
        
    step += 1
    
# Enter here exit cleanup code.

# Save sensors data as .csv and .pickle file
print("Saving results")
with open(SENSORS_DATA_FILENAME + ".pickle", "wb") as f: # save list of tuples as .pickle
    pickle.dump(sensors_data, f)
    

# TODO do we need to disable lidar, gps, robot here?