#This is a template code. Please save it in a proper .py file.
import rtmaps.types
import numpy as np
import rtmaps.core as rt 
import rtmaps.reading_policy 
from mpc_car_model import MPC_car_model # base class 
from datetime import datetime

# Python class that will be called from RTMaps.
class rtmaps_python(BaseComponent):
    def __init__(self):
        return
        

    def Dynamic(self):
        self.add_input("path", rtmaps.types.ANY) # define input
        self.add_input("state", rtmaps.types.ANY) # define input
        self.add_output("command", rtmaps.types.AUTO) # define output

# Birth() will be called once at diagram execution startup
    def Birth(self):
        self.mpc_car_model = MPC_car_model()
        self.timer = datetime.now()
        

# Core() is called every time you have a new input
    def Core(self):
        newPath = self.inputs["path"].ioelt # create an ioelt from the input
        newState = self.inputs["state"].ioelt

        self.mpc_car_model.set_state(newState)

        #See with seb if y/n
        newPath = self.mpc_car_model.local_to_global(newPath)
        self.mpc_car_model.set_state(newPath)

        date = datetime.now()
        diff = date - self.timer
        if self.mpc_car_model.path and self.mpc_car_model.x0 and diff.second > 4:
            self.timer = date
            command = self.mpc_car_model.run_MPC()
            out = self.outputs["out"].write(out) # and write it to the output
    
# Death() will be called once at diagram execution shutdown
    def Death(self):
        pass
