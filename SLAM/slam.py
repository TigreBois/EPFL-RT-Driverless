import time
import math
import numpy as np
from fastslam import FastSLAM
from graphslam import GraphSLAM

from .utils import normalizeAngle
from .rtmaps import rtmaps_python
from .tcplib import TCPLib

SLAMS = {
    "fastslam": FastSLAM,
    "graphslam": GraphSLAM
}

COM_SYSS = {
    "rtmaps":rtmaps_python,
    "tcplib": TCPLib
}

class SLAM:
    def __init__(self, slam_type="fastslam"):
        # SLAM_type = fast | graph
        if slam_type.startswith('graph'):
            self.slam_type = "graphslam"
        elif slam_type.startswith('fast'):
            self.slam_type = "fastslam"
        else:
            self.slam_type = "fastslam" # default

        self.slam = SLAMS[self.slam_type]()

        self.state = None
        self.map = None

        self.init_time = time.time()

    def run(self, perception, motion):

        z = np.zeros((3, 0))
        n_cones = len(perception)

        for i in range(n_cones):
            dx, dy, ci, color = perception[i]
            d = math.hypot(dx, dy)
            angle = normalizeAngle(math.atan2(dy, dx) - self.xTrue[2, 0])
            zi = np.array([d, normalizeAngle(angle), i]).reshape(3, 1)
            z = np.hstack((z, zi))
        
        xDR = np.array([motion[0], motion[1]])
        u = np.array([motion[2], motion[3]]).reshape(2, 1) #[v, yaw]

        if self.slam_type.startswith("graph"):
            self.state, self.map = self.slam.run()
            return self.state, self.map
        elif self.slam_type.startswith("fast"):
            self.state = self.slam.run(z, xDR, u, ci)
            return self.state
        else:
            print("SLAM type error")
        
        #TODO: detect loop closure to swhitch to graphslam

        return self.state, self.map

    def swith2graphSLAM(self):
        self.slam_type = "graphslam"
        self.slam = SLAMS[self.slam_type](self.map)

def main(communication = "rtmaps"):

    if communication in COM_SYSS.keys():
        COM_SYSS[communication]()
    else:
        print("Wrong communication system specified")
        rtmaps_python()


    print("Doing FastSLAM...")
    print("Lap detected and map created.")
    print("Running GraphSLAM...")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)