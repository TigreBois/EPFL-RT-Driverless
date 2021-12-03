from enum import Enum

class PhysicalModel(Enum):
    KINEMATIC = 1
    DYNAMIC = 2

class DrivingMode(Enum):
    STAGED = 0
    DRIVING = 1
    STOPPING = 2
    MISSION_FINISHED = 3

class Mission(Enum):
    ACCELERATION = 0
    INFINITE_STRAIGHT_LINE = 1
    SKIDPAD = 2
    TRACK_DRIVE = 3