import pandas as pd
from numpy.linalg import norm
from numpy import dot, arccos
from math import pi, cos, sin
from coordstransformslocal import webots2ISO, world2car

              
class Cones:
    """Helps to work with coordinates of cones"""
    def __init__(self, filename):
        """Loads cone coordiantes from csv file"""
        self.cones = pd.read_csv(filename).values
        print(f"Read {len(self.cones)} cone coordinates:") 
        # print(self.cones)
        
    def get_cones_car_ISO_coords(self, robot_coord_ISO, robot_orientation):
        """Convert cones from world Webots coordinates to car ISO coordinates"""
        cones_car_ISO = []
        
        for cone in self.cones:

            
            cone_pos_ISO = webots2ISO((cone[1], 0, cone[2]))
            cone_pos_ISO_car = world2car(cone_pos_ISO, robot_coord_ISO,
                                         robot_orientation)
                                         
            # cone[1], cone[2] are used just for debug purposes
            cones_car_ISO.append((cone[0], cone_pos_ISO_car[0],
                                            cone_pos_ISO_car[1],
                                            cone_pos_ISO_car[2], 
                                            cone[1], cone[2]))
             
        return cones_car_ISO
                                         
    def get_cones_in_front_car(self, robot_coord, robot_dir, distance):
        """Selects only cones whoch are in front of car and close to it"""
        cones_car_ISO = self.get_cones_car_ISO_coords(robot_coord, robot_dir)
         
        cones_in_front_within_within_dist = [cone for cone in cones_car_ISO 
                 if 0 < cone[0] < distance and -distance/2 < cone[1] < distance/2]
        
        return cones_in_front_within_within_dist                       
                                             
                        
        
        
        