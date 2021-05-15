"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

from PIL import Image
import cv2
import numpy as np
import os

IMAGE_FOLDER = "images"

# create the Robot instance.
robot = Robot()


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


camera = robot.getDevice("camera")
camera.enable(1000)
print("Starting camera")

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
step = 0
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    image = camera.getImageArray()
    # display the components of each pixel
    # for x in range(0,camera.getWidth()):
      # for y in range(0,camera.getHeight()):
        # red   = image[x][y][0]
        # green = image[x][y][1]
        # blue  = image[x][y][2]
        # gray  = (red + green + blue) / 3
        # print 'r='+str(red)+' g='+str(green)+' b='+str(blue)
        
    
    image = np.array(image)
    if np.mean(image) == 0:
        # Camera is still not ready
        continue
        
    print("Focal length is:", camera.getFocalLength())
    print("Focal distance is:", camera.getFocalDistance())
    
    filename = os.path.join(IMAGE_FOLDER, f"image_{step}.jpeg")

    # cv2
    # img_float32 = np.float32(image)
    # im_rgb = cv2.cvtColor(img_float32, cv2.COLOR_BGR2RGB)
    # img_rotate_90_clockwise = cv2.rotate(im_rgb, cv2.ROTATE_90_CLOCKWISE)
    # cv2.imwrite(filename, img_rotate_90_clockwise)

    # PIL
    # im = Image.fromarray(image)
    # im.save(filename)
    
    # Webots
    camera.saveImage(filename, 100)
    
    print(f"Image {filename} is saved")
    
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

    step += 1
# Enter here exit cleanup code.
