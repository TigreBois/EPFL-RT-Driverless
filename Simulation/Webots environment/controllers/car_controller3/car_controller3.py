from controller import PositionSensor
from vehicle import Driver
import numpy as np




def convert_radius_to_steering_angle(radius, wheelbase):
    return wheelbase / np.sqrt(radius ** 2 - wheelbase ** 2 / 4)


# create the Robot instance.
driver = Driver()

# get the time step of the current world.
timestep = int(driver.getBasicTimeStep())

wheelbase = 1.57 # [m]
tire_radius = 0.4604 # [m]
circle_radius = 5 # [m]
speed = 5 # [m/s]

angle = convert_radius_to_steering_angle(circle_radius, wheelbase)
print(angle)

# setting initial speed and steering angle
driver.setSteeringAngle(-angle)
driver.setCruisingSpeed(0)


T = 0

while driver.step() != -1:
    
    #print(flag == True)
    if T < 1500:
        print(T)
        T += timestep
    
    else:
        driver.setCruisingSpeed(speed)
        driver.setSteeringAngle(-angle)