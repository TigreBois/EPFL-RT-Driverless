import numpy as np
import pandas as pd
import pickle5 as pickle

import math

import matplotlib.pyplot as plt
show_animation = False

# Perception and motion input frequencies
PERCEPTION_FREQ = 30 #Hz
MOTION_FREQ = 200 #Hz
# TODO: We have to add the computational delay to calculate
# when the samples of each of the inputs match.
# The time tick will be the maximum between
DT = 1/min(PERCEPTION_FREQ, MOTION_FREQ) # time tick [s]

# Number of particles
N_PARTICLES = 100
NTH = N_PARTICLES / 1.5  # Number of particle for re-sampling

# Cone pose and state dimensions
CONE_DIMS = 2 # [x, y]
STATE_DIMS = 3 # [x, y, yaw]

# Distance parameters
MAX_RANGE = 20.0  # maximum observation range
M_DIST_TH = 2.0  # Threshold of Mahalanobis distance for data association.

# Fast SLAM covariance
Q = np.diag([3.0, np.deg2rad(10.0)]) ** 2
R = np.diag([0.8, np.deg2rad(20.0)]) ** 2

#  Simulation parameter
Q_sim = np.diag([0.3, np.deg2rad(2.0)]) ** 2
R_sim = np.diag([0.5, np.deg2rad(10.0)]) ** 2
OFFSET_YAWRATE_NOISE = 0.01

# Operating
SLAMMING = True # While receiving information


show_animation = True

class Particle:
    def __init__(self, N_CONES, x = 0, y = 0, yaw = 0):
        self.w = 1.0 / N_PARTICLES
        self.x = x
        self.y = y
        self.yaw = yaw
        # cones x-y positions
        self.cones = np.zeros((N_CONES, CONE_DIMS))
        # cones position covariance
        self.conesP = np.zeros((N_CONES * CONE_DIMS, CONE_DIMS))

def motion_model(x, u):
    """
    Helper function that returns the new predicted position for a particle
    :param x: vector with current particle position
    :param u: noise
    :F: 
    :returns: vector with the predicted position of the particle
    """
    F = np.identity(3)
    B = np.array([[DT * math.cos(x[2,0]), 0],
                  [DT * math.sin(x[2,0]), 0],
                  [0., DT]])
    x = F @ x + B @ u
    x[2, 0] = pi_2_pi(x[2, 0])
    return x

def predict_particles(particles, u):
    for i in range(N_PARTICLES):
        px = np.zeros((STATE_DIMS, 1))
        px[0, 0] = particles[i].x
        px[1, 0] = particles[i].y
        px[2, 0] = particles[i].yaw
        ud = u + (np.random.randn(1, 2) @ R).T  # add noise
        px = motion_model(px, ud)
        particles[i].x = px[0, 0]
        particles[i].y = px[1, 0]
        particles[i].yaw = px[2, 0]

    return particles

def pi_2_pi(angle):
    return (angle + math.pi)%(2*math.pi)-math.pi


def observation(xTrue, xd, u, frame):

    # calc true state
    xTrue = motion_model(xTrue, u)

    # add noise to range observation
    z = np.zeros((3, 0))
    for i in range(len(frame)):
        # print(frame[i].x, frame[i].y)
        # dx = TRACK[i, 0] - xTrue[0, 0]
        # dy = TRACK[i, 1] - xTrue[1, 0]
        dx = frame[i].x
        dy = frame[i].y
        dn = math.hypot(dx, dy) + np.random.randn() * Q_sim[0, 0]  # add noise
        anglen = pi_2_pi(math.atan2(dy, dx)) + np.random.randn() * Q_sim[1, 1]  # add noise
        zi = np.array([dn, pi_2_pi(anglen), i]).reshape(3, 1)
        z = np.hstack((z, zi))
    # for i in range(len(frame[:, 0])):
    #   print(i)


    # add noise to input
    ud1 = u[0, 0] + np.random.randn() * R_sim[0, 0]
    ud2 = u[1, 0] + np.random.randn() * R_sim[1, 1] #+ OFFSET_YAWRATE_NOISE
    ud = np.array([ud1, ud2]).reshape(2, 1)

    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud

def update_with_observation(particles, z):
    for iz in range(len(z[0, :])):

        coneid = int(z[2, iz])

        for ip in range(N_PARTICLES):
            # new cone
            if abs(particles[ip].cones[coneid, 0]) <= 0.01:
                particles[ip] = add_new_cone(particles[ip], z[:, iz], Q)
            # known cone
            else:
                w = compute_weight(particles[ip], z[:, iz], Q)
                particles[ip].w *= w
                particles[ip] = update_cone(particles[ip], z[:, iz], Q)

    return particles

def compute_weight(particle, z, Q):
    cone_id = int(z[2])
    xf = np.array(particle.cones[cone_id, :]).reshape(2, 1)
    Pf = np.array(particle.conesP[2 * cone_id:2 * cone_id + 2])
    zp, Hv, Hf, Sf = compute_jacobians(particle, xf, Pf, Q)
    dx = z[0:2].reshape(2, 1) - zp
    dx[1, 0] = pi_2_pi(dx[1, 0])

    try:
        invS = np.linalg.inv(Sf)
    except np.linalg.linalg.LinAlgError:
        print("singuler")
        return 1.0

    num = math.exp(-0.5 * dx.T @ invS @ dx)
    den = 2.0 * math.pi * math.sqrt(np.linalg.det(Sf))
    w = num / den

    return w

def compute_jacobians(particle, xf, Pf, Q):
    dx = xf[0, 0] - particle.x
    dy = xf[1, 0] - particle.y
    d2 = dx**2 + dy**2
    d = math.sqrt(d2)

    zp = np.array(
        [d, pi_2_pi(math.atan2(dy, dx) - particle.yaw)]).reshape(2, 1)

    Hv = np.array([[-dx / d, -dy / d, 0.0],
                   [dy / d2, -dx / d2, -1.0]])

    Hf = np.array([[dx / d, dy / d],
                   [-dy / d2, dx / d2]])

    Sf = Hf @ Pf @ Hf.T + Q

    return zp, Hv, Hf, Sf

def add_new_cone(particle, z, Q):

    r = z[0]
    b = z[1]
    cone_id = int(z[2])

    s = math.sin(pi_2_pi(particle.yaw + b))
    c = math.cos(pi_2_pi(particle.yaw + b))

    particle.cones[cone_id, 0] = particle.x + r * c
    particle.cones[cone_id, 1] = particle.y + r * s

    # covariance
    Gz = np.array([[c, -r * s],
                   [s, r * c]])

    particle.conesP[2 * cone_id:2 * cone_id + 2] = Gz @ Q @ Gz.T

    return particle

def update_KF_with_cholesky(xf, Pf, v, Q, Hf):
    PHt = Pf @ Hf.T
    S = Hf @ PHt + Q

    S = (S + S.T) * 0.5
    SChol = np.linalg.cholesky(S).T
    SCholInv = np.linalg.inv(SChol)
    W1 = PHt @ SCholInv
    W = W1 @ SCholInv.T

    x = xf + W @ v
    P = Pf - W1 @ W1.T

    return x, P

def update_cone(particle, z, Q):

    cone_id = int(z[2])
    xf = np.array(particle.cones[cone_id, :]).reshape(2, 1)
    Pf = np.array(particle.conesP[2 * cone_id:2 * cone_id + 2, :])

    zp, Hv, Hf, Sf = compute_jacobians(particle, xf, Pf, Q)

    dz = z[0:2].reshape(2, 1) - zp
    dz[1, 0] = pi_2_pi(dz[1, 0])

    xf, Pf = update_KF_with_cholesky(xf, Pf, dz, Q, Hf)

    particle.cones[cone_id, :] = xf.T
    particle.conesP[2 * cone_id:2 * cone_id + 2, :] = Pf

    return particle

def normalize_weight(particles):

    sumw = sum([p.w for p in particles])

    try:
        for i in range(N_PARTICLES):
            particles[i].w /= sumw
    except ZeroDivisionError:
        for i in range(N_PARTICLES):
            particles[i].w = 1.0 / N_PARTICLES

        return particles

    return particles


def resampling(particles):
    """
    low variance re-sampling
    """

    particles = normalize_weight(particles)

    pw = []
    for i in range(N_PARTICLES):
        pw.append(particles[i].w)

    pw = np.array(pw)

    Neff = 1.0 / (pw @ pw.T)  # Effective particle number

    if Neff < NTH:  # resampling
        wcum = np.cumsum(pw)
        base = np.cumsum(pw * 0.0 + 1 / N_PARTICLES) - 1 / N_PARTICLES
        resampleid = base + np.random.rand(base.shape[0]) / N_PARTICLES

        inds = []
        ind = 0
        for ip in range(N_PARTICLES):
            while ((ind < wcum.shape[0] - 1) and (resampleid[ip] > wcum[ind])):
                ind += 1
            inds.append(ind)

        tparticles = particles[:]
        for i in range(len(inds)):
            particles[i].x = tparticles[inds[i]].x
            particles[i].y = tparticles[inds[i]].y
            particles[i].yaw = tparticles[inds[i]].yaw
            particles[i].w = 1.0 / N_PARTICLES

    return particles

def gaussian(x, mu, sig):
    return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))


##############################################################################
#                                                                            #
##############################################################################

def fast_slam1(particles, u, z):
    particles = predict_particles(particles, u)

    particles = update_with_observation(particles, z)

    particles = resampling(particles)

    return particles

def calc_input(time):
    if time <= 7.5:  # wait at first
        v = 10.0
        yaw_rate = 0.0
    else:
        v = 10.0  # [m/s]
        yaw_rate = -0.8  # [rad/s]

    u = np.array([v, yaw_rate]).reshape(2, 1)

    return u

def calc_final_state(particles):
    xEst = np.zeros((STATE_DIMS, 1))

    particles = normalize_weight(particles)

    for i in range(N_PARTICLES):
        xEst[0, 0] += particles[i].w * particles[i].x
        xEst[1, 0] += particles[i].w * particles[i].y
        xEst[2, 0] += particles[i].w * particles[i].yaw

    xEst[2, 0] = pi_2_pi(xEst[2, 0])

    return xEst

def do_fast_slam(xEst, xTrue, xDR, hxEst, hxTrue, hxDR, u, motion, frame, particles, built_map):
        # xTrue, z, xDR, ud = observation(xTrue, xDR, u, TRACK) # meter dx y dx directamente en vez de track ////z = np.hstack((z, zi))
        xTrue, z, xDR, ud = observation(xTrue, xDR, u, frame) # frame --> [[cone1.dx, cone1.dy], [cone2.dx, cone2.dy], ...]
        # z = np.zeros((3, 0))
        # z = np.hstack(z, z)
        # ud = u
        # print(z)
        particles = fast_slam1(particles, ud, z)

        xEst = calc_final_state(particles)
        #print(xEst)

        x_state = xEst[0: STATE_DIMS]

        # store data history
        hxEst = np.hstack((hxEst, x_state))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))

        built_map.append([particles[0].cones[:, 0], particles[0].cones[:, 1]])

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event', lambda event:
                [exit(0) if event.key == 'escape' else None])

            for i in range(N_PARTICLES):
                plt.plot(particles[i].x, particles[i].y, ".r")
                plt.plot(particles[i].cones[:, 0], particles[i].cones[:, 1], "xb")
            for c in built_map:
                plt.plot(c[0], c[1], "xr")
            plt.plot(hxTrue[0, :], hxTrue[1, :], "-b")
            plt.plot(hxDR[0, :], hxDR[1, :], "-k")
            plt.plot(hxEst[0, :], hxEst[1, :], "-r")
            plt.plot(xEst[0], xEst[1], "xk")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)
        
        #####################################################################################
        #####################################################################################
        #####################################################################################
        # Send [[xEst], built_map]
        #####################################################################################
        #####################################################################################
        #####################################################################################

        return [[xEst], built_map]



class Cone:
    def __init__(self, x, y, ci, color):
        self.x = x
        self.y = y
        self.ci = ci
        self.color = color

def particles(motion)


def main(cones, motion):
    print(__file__ + " running...")

    time = 0.0

    # Real perception input, define Cone object from list of cones
    frame = [Cone(c[0],c[1],c[2],c[3]) for c in cones]

    # STATES = [] # [[x1 y1 v1 yaw1],[x2 y2 v2 yaw2],..., [xn yn vn yawn]] # Where n is the number of frames
    
    # State Vector [x y yaw v]'
    xEst = np.zeros((STATE_DIMS, 1))  # SLAM estimation
    xTrue = np.zeros((STATE_DIMS, 1))  # True state
    xDR = np.zeros((STATE_DIMS, 1))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue

    # while SLAMMING:
    built_map = []
    n_cones = len(frame)
    particles = [Particle(n_cones, motion[0], motion[1], motion[3]) for _ in range(N_PARTICLES)]
    time += 0.033
    u = calc_input(time)
    state, built_map = do_fast_slam(xEst, xTrue, xDR, hxEst, hxTrue, hxDR, u, motion, frame, particles, built_map)

    return built_map

if __name__ == '__main__':
    main(cone_data, motion_data)