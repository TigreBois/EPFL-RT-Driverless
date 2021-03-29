import gym
from gym import error, spaces, utils
from gym.spaces import Box, Discrete, Dict
from gym.utils import seeding

import numpy as np

from math import inf
import time, random

from vehicle import Driver

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

class RaceEnv(gym.Env):
    def __init__(self):
        super().__init__()
        
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
                
        self.driver = Driver()
        self.timestep = int(self.driver.getBasicTimeStep())
        self.sensors = self.init_sensors()
        self.stopped = True
        self.stepsPerEpisode = 200

    def init_sensors(self):
        lidar = self.driver.getDevice("lidar")
        #lidar.enable(self.timestep) # Are you sure?
        #lidar.enablePointCloud()
        
        gps = self.driver.getDevice("GPS")
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
        return sensor_data

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
