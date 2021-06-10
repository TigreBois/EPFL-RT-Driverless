#This is a template code. Please save it in a proper .py file.
import rtmaps.types
import numpy as np
import rtmaps.core as rt 
import rtmaps.reading_policy 
from rtmaps.base_component import BaseComponent # base class 

import time
from slam import SLAM

# Python class that will be called from RTMaps.
class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self) # call base class constructor
        self.slam = SLAM()

    def Dynamic(self):
        self.add_input("perception", rtmaps.types.ANY) # define input
        self.add_input("motion", rtmaps.types.ANY) # define input
    
        self.add_output("slam", rtmaps.types.AUTO) # define output

    # Birth() will be called once at diagram execution startup
    def Birth(self):
        # self.init = True
        # self.slam.start()
        print("Python Birth")

    # Core() is called every time you have a new input
    def Core(self):
        perception = self.inputs["perception"].ioelt # [[x1,y1,ci1,c1], [x2,y2,ci2,c2], ...]
        motion = self.inputs["motion"].ioelt # [x,y,v,yaw]  x, y, heading, velocity, acceleration, yaw rate

        self.slam.run(perception, motion)

        out = [self.slam.state, self.slam.map]
        self.outputs["slam"].write(out) # and write it to the output

    # Death() will be called once at diagram execution shutdown
    def Death(self):
        pass