import gym
from gym import error, spaces, utils
from gym.spaces import Box, Discrete, Dict
from gym.utils import seeding

import numpy as np

from math import inf, cos, pi, sqrt
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
CONE_DISTANCE = 8
CONE_PERCEPTION_DISTANCE = 20 # in meters
DO_PERCEPTION = False
BASE_SPEED = 2.0
START_THRESHOLD = 20

class RaceEnv(gym.Env):
    def __init__(self):
        super().__init__()
        
        """
        spaces = {
            "robot_speed": Box(low=-inf, high=inf,shape=(1,)),
            "yaw": Box(low=-inf, high=inf,shape=(1,)),
            "yaw_rate": Box(low=-inf, high=inf,shape=(1,)),
            "accelearation" :Box(low=-inf, high=inf,shape=(1,)),
            # "lidar": Box(low=-inf, high=inf,shape=(1,))
        }
        """

        wheelbase = 1.57 # [m]
        tire_radius = 0.4604 # [m]
        circle_radius = 5 # [m]
        speed = 5 # [m/s]
        angle = self.convert_radius_to_steering_angle(circle_radius, wheelbase)
        
        self.observation_space = Box(low=-inf, high=inf, shape=(8,))
        self.action_space = Box(low=np.array([0, -angle]), high=np.array([speed, angle]),shape=(2,)) # Cruising speed, Steering Angle
                
        self.driver = Driver()
 
        self.timestep = int(self.driver.getBasicTimeStep())
        self.sensors = self.init_sensors()
        self.stopped = True
        self.stepsPerEpisode = 200
        self.robot_coord_ISO = []
        self.cones_within_dist = []
        self.is_finished = False
        
        self.cones = Cones("../../../cone_coordinates.csv")

    def init_sensors(self):
        lidar = self.driver.getDevice("lidar")
        lidar.enable(self.timestep) # Are you sure?
        lidar.enablePointCloud()
        
        gps = self.driver.getDevice("gps")
        gps.enable(self.timestep)
        
        gyro = self.driver.getDevice("gyro")
        gyro.enable(self.timestep)
        # gyro.xAxis, gyro.zAxis = False, False # We need only yaw rate
        
        imu = self.driver.getDevice("imu")
        imu.enable(self.timestep)
        # imu.xAxis, imu.zAxis = False, False # We need only yaw angle
        
        accelerometer = self.driver.getDevice("accelerometer")
        accelerometer.enable(self.timestep)
        
        sensors = {"lidar": lidar, "gps": gps, "gyro": gyro, "imu": imu, "accelerometer": accelerometer}
        return sensors

    def step(self, action):
        self.apply_action(action)
        driver_done = self.driver.step()
        observations = self.get_observations()
        return observations, self.get_reward(observations), driver_done == -1 or self.is_done(), self.get_info()

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
        
        sensor_data = {"robot_coord": gps_values, "robot_speed": speed, "yaw": yaw,
                        "acceleration": acceleration, "yaw_rate": yaw_rate}
        return sensor_data    
        

    def get_observations(self):
        sensor_data = self.get_sensor_data(self.sensors)

    
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
        cones_car_ISO = self.cones.get_cones_car_ISO_coords(robot_coord_ISO, robot_orientation_ISO)
        
        cones_car_ISO = [(cone[0], cone[1], cone[2], cone[3],
                        cone[4] / CONE_SCALING, cone[5] / CONE_SCALING) for cone in cones_car_ISO]
        # print("Cones in car ISO coordinates:")
        # for cone in cones_car_ISO:
            # print(cone)
            
        cones_within_dist = [cone for cone in cones_car_ISO
            if 0 < cone[1] < CONE_PERCEPTION_DISTANCE and -CONE_PERCEPTION_DISTANCE < cone[2] < CONE_PERCEPTION_DISTANCE]
        
        # print("Visible cones in car ISO coordinates")

        #for cone in cones_within_dist:
            #print(cone)
        
        blue_cones_dist, yellow_cones_dist = self.get_cones_dist(cones_within_dist, robot_coord_ISO)
        
        self.robot_coord_ISO = robot_coord_ISO
        self.cones_within_dist = cones_within_dist

        sensors_data_new = [robot_speed,
                            yaw_ISO,
                            yellow_cones_dist[0],yellow_cones_dist[1], yellow_cones_dist[2],
                            blue_cones_dist[0], blue_cones_dist[1], blue_cones_dist[2]]

        print(f"Observation: {sensors_data_new}")
        return sensors_data_new
    
    def get_cones_dist(self, cones_within_dist, robot_coord_ISO):
        blue_cones_dist = [self.norm(cone[1], cone[2]) for cone in cones_within_dist if cone[0]=="blue"]
    
        yellow_cones_dist = [self.norm(cone[1], cone[2]) for cone in cones_within_dist if cone[0]=="yellow"]

        blue_cones_dist.sort()
        yellow_cones_dist.sort()

        while len(blue_cones_dist)<3:
            blue_cones_dist.append(CONE_PERCEPTION_DISTANCE)
        while len(yellow_cones_dist)<3:
            yellow_cones_dist.append(CONE_PERCEPTION_DISTANCE)
        
        return blue_cones_dist, yellow_cones_dist

    def norm(self, x, y):
        return self.distance(x,y, 0, 0)
    def distance(self, x, y, car_x, car_y):
        return sqrt((x-car_x)**2+(y-car_y)**2)

    def reset(self):
        print("************ Resetting ****************")
        self.resetPosition()
        self.sensors = self.init_sensors()
        self.stopped = True
        self.is_finished = False
        self.driver.step()
        return self.get_observations()

    def resetPosition(self):
        robot_node = self.driver.getSelf()
        trans_field = robot_node.getField("translation")
        INITIAL = [21.8982, 0.225869, -51.6669]
        trans_field.setSFVec3f(INITIAL)
        rotation_field = robot_node.getField("rotation")
        INITIAL_ROT = [0.0,1.0,0.0, 0.0]
        rotation_field.setSFRotation(INITIAL_ROT)
        self.driver.setSteeringAngle(0)
        self.driver.setCruisingSpeed(0)
        self.driver.simulationResetPhysics()

    def get_reward(self, observations):
        robot_speed = observations[0]
        if robot_speed < 0.1:
            return -1/10000
        if len(self.cones_within_dist)==0 or len(self.robot_coord_ISO)== 0:
            return 1/10000

        """
        for cone in self.cones_within_dist:
            if abs(cone[1]-self.robot_coord_ISO[0]) < 0.1 or abs(cone[2]-self.robot_coord_ISO[1])<0.1:
                self.is_done = True
                return -1
        """
        if observations[2]+observations[3] <= CONE_DISTANCE or observations[5]+observations[6] <= CONE_DISTANCE:
            self.is_finished = True
            print("IS DONE")
            return -100
        return 1

    def is_done(self):
        #return (self.stopped and self.driver.getTime() > START_THRESHOLD) or self.is_finished
        return self.is_finished
        #return False

    def convert_radius_to_steering_angle(self, radius, wheelbase):
        return wheelbase / np.sqrt(radius ** 2 - wheelbase ** 2 / 4)

    def apply_action(self, action):
        wheelbase = 1.57 # [m]
        tire_radius = 0.4604 # [m]
        circle_radius = 5 # [m]
        speed = 5 # [m/s]
        angle = self.convert_radius_to_steering_angle(circle_radius, wheelbase)
        print(angle)
        launch_speed = min(abs(float(action[0])),speed)
        print("launching speed", launch_speed)
        self.driver.setCruisingSpeed(launch_speed)
        launch_angle = max(min(float(action[1]), angle), -angle)
        print(launch_angle, action)
        self.driver.setSteeringAngle(launch_angle)

    def render(self, mode='human', close=False):
        print("render() is not used")

    def get_info(self):
        return {}

##todo never be done