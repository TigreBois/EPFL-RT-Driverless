import gym
from gym import error, spaces, utils
from gym.spaces import Box, Discrete, Dict
from gym.utils import seeding

import numpy as np

from math import inf, cos, pi
import time, random

from vehicle import Driver
from controller import GPS, Gyro, InertialUnit, Accelerometer # Motion estimation
from controller import Robot, Motor, Lidar, LidarPoint 
from cones import Cones
from coordstransformslocal import webots2ISO, world2car

from utilities import normalizeToRange, plotData


"""
    3 Actions:
        - LEFT (-1)
        - RIGTH (1)
        - NOTHING (0)
    Observation Space:
        - 3 LIDARS in angles 45, 90, 135
    Reward:
        - (1) if no collision
        - (-1) if collision
"""

sensors = ["robot_coord", "robot_speed", "yaw", "yaw_rate", "acceleration"]
CONE_SCALING = 0.5 # How cones were scaled when created (Will be used to ease search of cones in Webots window)
CONE_PERCEPTION_DISTANCE = 20 # in meters
DO_PERCEPTION = False
BASE_SPEED = 2.0

class CustomEnv(gym.Env):
    def __init__(self, driver):
        super().__init__()
        
        self.driver = driver
        spaces = {
            "robot_coord": Box(low=-inf, high=inf,shape=(2,)),
            "robot_speed": Box(low=-inf, high=inf,shape=(1,)),
            "yaw": Box(low=-inf, high=inf,shape=(1,)),
            "yaw_rate": Box(low=-inf, high=inf,shape=(1,)),
            "accelearation" :Box(low=-inf, high=inf,shape=(1,)),
            # "lidar": Box(low=-inf, high=inf,shape=(1,))
        }
        self.observation_space = Dict(spaces)
        self.action_space = Box(low=-inf, high=inf,shape=(2,)) # Cruising speed, Steering Angle

 
        self.timestep = int(self.driver.getBasicTimeStep())
        self.sensors = self.init_sensors()
        self.stopped = True
        self.stepsPerEpisode = 200
        
        self.cones = Cones("../../../cone_coordinates.csv")

    def init_sensors(self):
        lidar = self.driver.getDevice("lidar")
        #lidar.enable(self.timestep) # Are you sure?
        #lidar.enablePointCloud()
        
        gps = self.driver.getDevice("gps")
        gps.enable(self.timestep)
        
        gyro = self.driver.getDevice("gyro")
        #gyro.enable(self.timestep)
        # gyro.xAxis, gyro.zAxis = False, False # We need only yaw rate
        
        imu = self.driver.getDevice("imu")
        #imu.enable(self.timestep)
        # imu.xAxis, imu.zAxis = False, False # We need only yaw angle
        
        accelerometer = self.driver.getDevice("accelerometer")
        #accelerometer.enable(self.timestep)
        
        front_sensor = self.driver.getDevice("ds_front")
        #front_sensor.enable(self.timestep)

        sensors = {"lidar": lidar, "gps": gps, "gyro": gyro, "imu": imu, "accelerometer": accelerometer}
        return sensors

    def step(self, action):
        self.apply_action(action)
        return self.get_observations(), self.get_reward, self.is_done, self.get_info

    def get_sensor_data(self, sensors):
        """
        gps_values = sensors["gps"].getValues()
        #print("Robot coordinates:", gps_values)
        
        speed = sensors["gps"].getSpeed()
        #print("Robot speed", speed)
        
        # roll_np.pitch_yaw = sensors["imu"].getRollnp.pitchYaw() # Need [2] - index of yaw
        # yaw = roll_np.pitch_yaw[2]
        yaw = 0
        #print("Yaw angle is", roll_np.pitch_yaw, "=>", yaw)
        
        acceleration = 0
        #acceleration_values = sensors["accelerometer"].getValues()
        # Need to check calculations:
        #acceleration = acceleration_values[0] * np.cos(yaw) + acceleration_values[2] * np.cos(yaw + np.pi/2)
        #print("Acceleration is", acceleration_values, "=>", acceleration)
        angle_velocity = 0
        #angle_velocity = sensors["gyro"].getValues() # Need [1] - index of yaw
        yaw_rate = 0
        #yaw_rate = angle_velocity[1]
        #print("Yaw rate is", angle_velocity, "=>", yaw_rate)

        #lidar_values = sensors["lidar"].getValues()
        lidar_values = 0
        #lidar_values = 0
        print("Lidar values", gps_values)
        
        sensor_data = {"robot_coord": gps_values, "robot_speed": speed, "yaw": yaw,
                        "acceleration": acceleration, "yaw_rate": yaw_rate, "lidar":lidar_values}
                        
                   """     
        #########
        
        # Work with sensors 

        robot_coord = sensors['gps'].getValues()
        print("Robot coordinates:", robot_coord)
        
        robot_speed = sensors['gps'].getSpeed()
        print("Robot speed", robot_speed)
        
        robot_orientation = sensors['imu'].getRollPitchYaw() # Need [2] - index of yaw
        yaw = robot_orientation[2]
        print("Yaw angle is", yaw)
        
        acceleration = sensors['accelerometer'].getValues()
        # Need to check calculations:
        acceleration_dir = acceleration[0] * cos(yaw) + acceleration[2] * cos(yaw + pi/2)
        print("Acceleration is", acceleration)
        
        angle_velocity = sensors['gyro'].getValues() # Need [1] - index of yaw
        yaw_rate = angle_velocity[1]
        print("Yaw rate is", yaw_rate)
        
        robot_coord_ISO = webots2ISO(robot_coord)
        print("Robot coords ISO:", robot_coord_ISO)
        # yaw_ISO = yaw + pi/2 # For previous version of car (New version is rotated)
        yaw_ISO = yaw + pi/2
        print("yaw ISO:", yaw_ISO)
        robot_orientation_ISO = (0, 0, yaw_ISO)
        cones_car_ISO = self.cones.get_cones_car_ISO_coords(robot_coord_ISO, robot_orientation_ISO)
        cones_car_ISO = [(cone[0], cone[1], cone[2], cone[3],
                          cone[4] / CONE_SCALING, cone[5] / CONE_SCALING) for cone in cones_car_ISO]
        print("Cones in car ISO coordinates:")
        #for cone in cones_car_ISO:
            #print(cone)
            
        cones_within_dist = [cone for cone in cones_car_ISO
            if 0 < cone[1] < CONE_PERCEPTION_DISTANCE and -CONE_PERCEPTION_DISTANCE < cone[2] < CONE_PERCEPTION_DISTANCE]
        print("Visible cones in car ISO coordinates")
        #for cone in cones_within_dist:
            #print(cone)
            
            
        
        
        #########
        #return sensor_data

    def get_observations(self):
        sensor_data = self.get_sensor_data(self.sensors)
        obs = []
        for sensor in sensors:
            obs.append(sensor_data[sensor])
        return obs

    def reset(self):
        self.sensors = self.init_sensors()
        self.stopped = True

    def get_reward(self, action=None):
        return not self.stopped

    def is_done(self):
        return self.stopped and self.driver.getTime() > START_THRESHOLD

    def apply_action(self, action):
        self.driver.setCruisingSpeed(float(action[0]))
        self.driver.setSteeringAngle(float(action[1]))

    def render(self, mode='human', close=False):
        print("render() is not used")

    def get_info(self):
        return None
