import rtmaps.types
import numpy as np
import rtmaps.core as rt
import rtmaps.reading_policy
import rtmaps.base_component.BaseComponent as BaseComponent

import time


class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)

    def Dynamic(self):
        self.add_input("in", rtmaps.types.ANY)
        self.add_output("out", rtmaps.types.AUTO)

    def Birth(self):
        self.inputList = []
        print("Python Birth")

    def Core(self):
        out = self.inputs["in"].ioelt
        self.inputList.append(out)
        print("Input in source: " + str(out))
        print("total inputs:" + str(self.inputList))
        #time.sleep(30)
        self.outputs["out"].write(out)

    def Death(self):
        pass