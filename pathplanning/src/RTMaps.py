# This is a template code. Please save it in a proper .py file.
import rtmaps.types
import numpy as np
import rtmaps.core as rt
import rtmaps.reading_policy
from rtmaps.base_component import BaseComponent  # base class

# Python class that will be called from RTMaps.
from pathplanning.src.MainCode import middle_path
from pathplanning.src.MainCode import local_to_global


class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)  # call base class constructor

    def Dynamic(self):
        self.add_input("yellow_cones", rtmaps.types.ANY)  # define input
        self.add_input("blue_cones", rtmaps.types.ANY)  # define input
        self.add_input("car", rtmaps.types.ANY)  # define input

        self.add_output("middle_path", rtmaps.types.AUTO)  # define output
        self.add_output("yellow_track", rtmaps.types.AUTO)  # define output
        self.add_output("blue_track", rtmaps.types.AUTO)  # define output

    # Birth() will be called once at diagram execution startup
    def Birth(self):
        print("Python Birth")

    # Core() is called every time you have a new input
    def Core(self):
        yellow = self.inputs["yellow_cones"].ioelt  # create an ioelt from the input
        blue = self.inputs["blue_cones"].ioelt  # create an ioelt from the input
        car = self.inputs["car"].ioelt  # create an ioelt from the input

        path, yellow_track, blue_track = middle_path(yellow, blue)

        yellow_track = local_to_global(yellow_track, car)
        blue_track = local_to_global(blue_track, car)

        self.outputs["middle_path"].write(path)  # and write it to the output
        self.outputs["yellow_track"].write(yellow_track)  # and write it to the output
        self.outputs["blue_track"].write(blue_track)  # and write it to the output

    # Death() will be called once at diagram execution shutdown
    def Death(self):
        pass
