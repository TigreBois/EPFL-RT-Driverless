import math
import numpy as np

def normalizeAngle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def motion_model(x, u, dt):
    """
    Helper function that returns the new predicted position for a particle
    :param x: vector with current particle position
    :param u: noise
    :F: 
    :returns: vector with the predicted position of the particle
    """
    F = np.identity(3)
    B = np.array([[dt * math.cos(x[2, 0]), 0],
                  [dt * math.sin(x[2, 0]), 0],
                  [0., dt]])
    x = F @ x + B @ u
    x[2, 0] = normalizeAngle(x[2, 0])
    return x