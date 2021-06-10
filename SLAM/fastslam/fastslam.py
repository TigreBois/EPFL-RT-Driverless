import time
import math
import numpy as np
from slam.utils import normalizeAngle
from .predict import predict_particles
from .update import update_with_observation
from .resampling import get_final_state, resampling

class FastSLAM:
  def __init__(
    self, 
    version=2,
    state_dims=3,
    n_particles=100,
    perception_dt=.033,
    odometry_dt=.005,
    pose_var=6.,
    yaw_var=.1,
    visualize=False
  ):
    self.version = version
    self.state_dims = state_dims

    self.car_sate = np.zeros((self.state_dims,1))
    self.map = []

    self.visualize = visualize
    self.n_particles = n_particles
    self.n_resampled = n_particles / 1.5

    self.R = np.diag([pose_var, yaw_var])**2 #How much we trust

    self.odometry_dt = odometry_dt
    self.perception_dt = perception_dt

    sPos     = 0.5*8.8*odometry_dt**2  # assume 8.8m/s2 as maximum acceleration, forcing the vehicle (Alex D.)
    sCourse  = 0.1*odometry_dt # assume 0.1rad/s as maximum turn rate for the vehicle (Alex D.)
    self.Q = np.diag([sPos, sCourse])**2


    ####################################################
    self.xEst = np.zeros((self.state_dims, 1)) # SLAM estimation
    self.xDR = np.zeros((self.state_dims, 1)) # Dead reckoning

    self.hxEst = self.xEst # history SLAM estimation
    self.hxDR = self.xDR # history Dead reckoning

    self.init_time = time.time()
    ######################################################


  def run(self, z, xDR, u, ci):
    n_cones = z.shape
    self.particles = [Particle(n_cones, *xDR, u[1,0])
                        for _ in range(self.n_particles)]


    particles = predict_particles(self.particles, u, self.R)

    particles = update_with_observation(particles, z, self.Q, ci, self.version)

    particles = resampling(particles, self.n_resampled)


    self.car_state = get_final_state(particles, self.state_dims)
    self.map.append([particles[0].cones[:, 0], particles[0].cones[:, 1]])
    
    return self.car_state, self.map


class Particle:
  def __init__(self, n_cones, x=0, y=0, yaw=0, n_particles=100, n_states=3):
    self.w = 1.0 / n_particles
    self.x = x
    self.y = y
    self.yaw = yaw
    self.P = np.eye(n_states)

    # cones x-y positions
    self.cones = np.zeros((n_cones, n_states))
    
    # cones position covariance
    self.conesP = np.zeros((n_cones * n_states, n_states))

  def set_pose(self, x, y, yaw):
      self.x = x
      self.y = y
      self.yaw = yaw
      return self