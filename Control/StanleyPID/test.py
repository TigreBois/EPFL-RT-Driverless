import numpy as np
from StanleyPID.StanleyPIDPython import wrapperStanleyControl
from StanleyPID.common import *

referencePath = np.c_[np.linspace(0,250,100), np.zeros(100)].T
initialState = np.zeros(6)
controlInputs = wrapperStanleyControl(referencePath, initialState, mission=Mission.ACCELERATION)
