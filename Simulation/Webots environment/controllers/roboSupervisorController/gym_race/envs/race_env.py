import gym
from gym import error, spaces, utils
from gym.utils import seeding
import pygame
import time
import random
import math
from math import tan
from math import sqrt
from utilities import normalizeToRange, plotData
import random
import numpy as np
from gym.spaces import Box, Discrete
from math import inf
from vehicle import Driver

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


# Advance step by step
class RaceEnv(gym.Env):
    def __init__(self):
        super().__init__()
        
        spaces = {
            "robot_coord": Box(low=-inf, high=inf,shape=(2,)),
            "robot_speed": Box(low=-inf, high=inf,shape=(1,)),
            "yaw": Box(low=-inf, high=inf,shape=(1,)),
            "yaw_rate": Box(low=-inf, high=inf,shape=(1,)),
            "accelearation" :Box(low=-inf, high=inf,shape=(1,)),
            "lidar": Box(low=-inf, high=inf,shape=(1,))
        }
        self.observation_space = Box(low=-inf, high=inf,shape=(2,)) # ["robot_coord","robot_speed","yaw","acceleration","yaw_rate","lidar"
        self.action_space = Box(low=-inf, high=inf,shape=(2,)) # Cruising speed, Steering Angle
        
        #self.robot = self.getSelf()  # Grab the robot reference from the supervisor to access various robot methods
        
        self.robot = Driver()
        self.sensors = self.init_sensors(1) # how to get timestep? self.init_sensors(self.robot.timestep)
        self.stopped = True

        self.stepsPerEpisode = 200  # Max number of steps per episode
        self.episodeScore = 0  # Score accumulated during an episode
        self.episodeScoreList = []  # A list to save all the episode scores, used to check if task is solved

    def init_sensors(self, timestep):
        lidar = self.robot.getDevice("lidar")
        lidar.enable(timestep) # Are you sure?
        lidar.enablePointCloud()
        
        gps = self.robot.getDevice("gps")
        gps.enable(timestep)
        
        gyro = self.robot.getDevice("gyro")
        gyro.enable(timestep)
        # gyro.xAxis, gyro.zAxis = False, False # We need only yaw rate
        
        imu = self.robot.getDevice("imu")
        imu.enable(timestep)
        # imu.xAxis, imu.zAxis = False, False # We need only yaw angle
        
        accelerometer = self.robot.getDevice("accelerometer")
        accelerometer.enable(timestep)

        sensors = {"lidar": lidar, "gps": gps, "gyro": gyro, "imu": imu, "accelerometer": accelerometer}
        return sensors

    def step(self):
        return self.get_observations(), self.get_reward, self.is_done, self.get_info

    def get_sensor_data(self, sensors):
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

        lidar_values = sensors["lidar"].getValues()
        print("Lidar values", lidar_values)
        
        sensor_data = {"robot_coord": gps_values, "robot_speed": speed, "yaw": yaw,
                        "acceleration": acceleration, "yaw_rate": yaw_rate, "lidar":lidar_values}
        return sensor_data

    def get_observations(self):
        """
        # Position on z axis
        cartPosition = normalizeToRange(self.robot.getPosition()[2], -0.4, 0.4, -1.0, 1.0)
        # Linear velocity on z axis
        cartVelocity = normalizeToRange(self.robot.getVelocity()[2], -0.2, 0.2, -1.0, 1.0, clip=True)
        # Angular velocity x of endpoint
        endpointVelocity = normalizeToRange(self.poleEndpoint.getVelocity()[3], -1.5, 1.5, -1.0, 1.0, clip=True)
        return [cartPosition, cartVelocity, poleAngle, endpointVelocity]
        """
        sensor_data = get_sensor_data(self.sensors)
        return [sensor_data["robot_coord"], sensor_data["robot_speed"], sensor_data["yaw"], sensor_data["acceleration"], sensor_data["yaw_rate"], sensor_data["lidar"]]
        
    def reset(self):
        self.sensors = self.init_sensors(1) # how to get timestep? self.init_sensors(self.robot.timestep)
        self.stopped = True

        self.stepsPerEpisode = 200  # Max number of steps per episode
        self.episodeScore = 0  # Score accumulated during an episode
        self.episodeScoreList = []  # A list to save all the episode scores, used to check if task is solved


    def get_reward(self, action=None):
        return 1

    def is_done(self):
        return self.stopped and self.robot.getTime() > START_THRESHOLD

    def solved(self):
        if len(self.episodeScoreList) > 100:  # Over 100 trials thus far
            if np.mean(self.episodeScoreList[-100:]) > 195.0:  # Last 100 episodes' scores average value
                return True
        return False

    def get_default_observation(self):
        return [0.0 for _ in range(self.observation_space.shape[0])]

    def apply_action(self, action):
        self.setCruisingSpeed(action[0])
        self.setSteeringAngle(-action[1]) # TODO Don't think we need the negative


    def render(self, mode='human'):
        print("render() is not used")

    def get_info(self):
        return None

