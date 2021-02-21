import pandas as pd
from numpy.linalg import norm
from numpy import dot, arccos
from math import pi, cos, sin
from coordstransformslocal import webots2ISO, world2car

X_AXIS = (1, 0)
DISTANCE_BETWEEN_GPS_AND_ROBOT_FRONT_CENTER = 0.1
ROBOT_WIDTH = 0.1

def rotate_vector(vec, angle):
    # angle in radians
    x, y = vec
    x_new = cos(angle) * x - sin(angle) * y
    y_new = sin(angle) * x - cos(angle) * y
    
    return (x_new, y_new)
    
def is_object_in_front(robot_dir, object_pos):
    robot_dir_norm = robot_dir / norm(robot_dir)
    
    cone_dir = object_pos # Since object positions are already realtive to robot position
    cone_dir_norm = cone_dir / norm(cone_dir)
    dot_product = dot(robot_dir_norm, cone_dir_norm)
    angle = arccos(dot_product)
    
    return angle < pi / 2
              
class Cones:
    """Helps to work with coordinates of cones"""
    def __init__(self, filename):
        """Loads cone coordiantes from csv file"""
        self.cones = pd.read_csv(filename).values
        print(f"Read {len(self.cones)} cone coordinates:") 
        print(self.cones)
        
    def get_cones_car_ISO_coords(self, robot_coord_ISO, robot_orientation):
        """Convert cones from world Webots coordinates to car ISO coordinates"""
        cones_car_ISO = []
        
        for cone in self.cones:

            
            cone_pos_ISO = webots2ISO((cone[1], 0, cone[2]))
            cone_pos_ISO_car = world2car(cone_pos_ISO, robot_coord_ISO,
                                         robot_orientation)
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
        
    
    # Outdated
    def get_cones_relative_coordinates(self, robot_coord):
        """Returns coordinates of cones relatively robot coordiantes"""
        # rel_coords = [(color,
                       # x - robot_coord[0],
                       # z - robot_coord[2]) for (color, x, z) in self.cones]
        rel_coords = [(color,
                       x - robot_coord[0],
                       z - robot_coord[2], 
                       x, z) for (color, x, z) in self.cones]
        return rel_coords
        
    def get_cones_within_distance(self, robot_coord, distance):
        """Returns relative distances of cones which are within particular distance to robot"""
        cones_rel_coords = self.get_cones_relative_coordinates(robot_coord)
        
        cones_within_distance = [cone for cone in cones_rel_coords \
               if norm((cone[1], cone[2])) < distance]
               
        return cones_within_distance
        
         
    def get_cones_within_dist_in_front(self, robot_coord, robot_dir_angle, distance):
        """Returns relative distances of cones which are i front of robot and within 
        particular distance to it"""
        cones_within_distance = self.get_cones_within_distance(robot_coord, distance)
        
        robot_dir = rotate_vector(X_AXIS, robot_dir_angle)
        
        cones_within_dist_in_front = [cone for cone in cones_within_distance
                                          if is_object_in_front(robot_dir, (cone[1], cone[2]))]
                                                                
        return cones_within_dist_in_front
                                             
                        
        
        
        