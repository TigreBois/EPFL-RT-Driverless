import copy
import itertools
import math
import pickle5 as pickle

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as Rot



#  Simulation parameter
Q_sim = np.diag([0.2, np.deg2rad(1.0)]) ** 2
R_sim = np.diag([0.1, np.deg2rad(10.0)]) ** 2

DT = 2.0  # time tick [s]
SIM_TIME = 100.0  # simulation time [s]
MAX_RANGE = 30.0  # maximum observation range
STATE_DIMS = 3  # State size [x,y,yaw]

# Covariance parameter of Graph Based SLAM
C_SIGMA1 = 0.1
C_SIGMA2 = 0.1
C_SIGMA3 = np.deg2rad(1.0)

MAX_ITR = 20  # Maximum iteration

show_graph_d_time = 20.0  # [s]
show_animation = True


class Edge:

    def __init__(self):
        self.e = np.zeros((3, 1))
        self.omega = np.zeros((3, 3))  # information matrix
        self.d1 = 0.0
        self.d2 = 0.0
        self.yaw1 = 0.0
        self.yaw2 = 0.0
        self.angle1 = 0.0
        self.angle2 = 0.0
        self.id1 = 0
        self.id2 = 0

class Cone:
    def __init__(self, x, y, ci, color):
        self.x = x
        self.y = y
        self.ci = ci
        self.color = color

def cal_observation_sigma():
    sigma = np.zeros((3, 3))
    sigma[0, 0] = C_SIGMA1 ** 2
    sigma[1, 1] = C_SIGMA2 ** 2
    sigma[2, 2] = C_SIGMA3 ** 2

    return sigma


def calc_rotational_matrix(angle):
    return Rot.from_euler('z', angle).as_matrix()


def calc_edge(x1, y1, yaw1, x2, y2, yaw2, d1,
              angle1, d2, angle2, t1, t2):
    edge = Edge()

    tangle1 = pi_2_pi(yaw1 + angle1)
    tangle2 = pi_2_pi(yaw2 + angle2)
    tmp1 = d1 * math.cos(tangle1)
    tmp2 = d2 * math.cos(tangle2)
    tmp3 = d1 * math.sin(tangle1)
    tmp4 = d2 * math.sin(tangle2)

    edge.e[0, 0] = x2 - x1 - tmp1 + tmp2
    edge.e[1, 0] = y2 - y1 - tmp3 + tmp4
    edge.e[2, 0] = 0

    Rt1 = calc_rotational_matrix(tangle1)
    Rt2 = calc_rotational_matrix(tangle2)

    sig1 = cal_observation_sigma()
    sig2 = cal_observation_sigma()

    edge.omega = np.linalg.inv(Rt1 @ sig1 @ Rt1.T + Rt2 @ sig2 @ Rt2.T)

    edge.d1, edge.d2 = d1, d2
    edge.yaw1, edge.yaw2 = yaw1, yaw2
    edge.angle1, edge.angle2 = angle1, angle2
    edge.id1, edge.id2 = t1, t2

    return edge


def calc_edges(x_list, z_list):
    edges = []
    cost = 0.0
    z_ids = list(itertools.combinations(range(len(z_list)), 2))

    for (t1, t2) in z_ids:
        x1, y1, yaw1 = x_list[0, t1], x_list[1, t1], x_list[2, t1]
        x2, y2, yaw2 = x_list[0, t2], x_list[1, t2], x_list[2, t2]

        if z_list[t1] is None or z_list[t2] is None:
            continue  # No observation

        for iz1 in range(len(z_list[t1][:, 0])):
            for iz2 in range(len(z_list[t2][:, 0])):
                if z_list[t1][iz1, 3] == z_list[t2][iz2, 3]:
                    d1 = z_list[t1][iz1, 0]
                    angle1, phi1 = z_list[t1][iz1, 1], z_list[t1][iz1, 2]
                    d2 = z_list[t2][iz2, 0]
                    angle2, phi2 = z_list[t2][iz2, 1], z_list[t2][iz2, 2]

                    edge = calc_edge(x1, y1, yaw1, x2, y2, yaw2, d1,
                                     angle1, d2, angle2, t1, t2)

                    edges.append(edge)
                    cost += (edge.e.T @ edge.omega @ edge.e)[0, 0]

    print("cost:", cost, ",n_edge:", len(edges))
    return edges


def calc_jacobian(edge):
    t1 = edge.yaw1 + edge.angle1
    A = np.array([[-1.0, 0, edge.d1 * math.sin(t1)],
                  [0, -1.0, -edge.d1 * math.cos(t1)],
                  [0, 0, 0]])

    t2 = edge.yaw2 + edge.angle2
    B = np.array([[1.0, 0, -edge.d2 * math.sin(t2)],
                  [0, 1.0, edge.d2 * math.cos(t2)],
                  [0, 0, 0]])

    return A, B


def fill_H_and_b(H, b, edge):
    A, B = calc_jacobian(edge)

    id1 = edge.id1 * STATE_DIMS
    id2 = edge.id2 * STATE_DIMS

    H[id1:id1 + STATE_DIMS, id1:id1 + STATE_DIMS] += A.T @ edge.omega @ A
    H[id1:id1 + STATE_DIMS, id2:id2 + STATE_DIMS] += A.T @ edge.omega @ B
    H[id2:id2 + STATE_DIMS, id1:id1 + STATE_DIMS] += B.T @ edge.omega @ A
    H[id2:id2 + STATE_DIMS, id2:id2 + STATE_DIMS] += B.T @ edge.omega @ B

    b[id1:id1 + STATE_DIMS] += (A.T @ edge.omega @ edge.e)
    b[id2:id2 + STATE_DIMS] += (B.T @ edge.omega @ edge.e)

    return H, b


def graph_based_slam(x_init, hz):
    print("start graph based slam")

    z_list = copy.deepcopy(hz)

    x_opt = copy.deepcopy(x_init)
    nt = x_opt.shape[1]
    n = nt * STATE_DIMS

    for itr in range(MAX_ITR):
        edges = calc_edges(x_opt, z_list)

        H = np.zeros((n, n))
        b = np.zeros((n, 1))

        for edge in edges:
            H, b = fill_H_and_b(H, b, edge)

        # to fix origin
        H[0:STATE_DIMS, 0:STATE_DIMS] += np.identity(STATE_DIMS)

        try:
            dx = - np.linalg.inv(H) @ b
        except ValueError:
            dx = - b @ H

        for i in range(nt):
            x_opt[0:3, i] += dx[i * 3:i * 3 + 3, 0]

        diff = dx.T @ dx
        print("iteration: %d, diff: %f" % (itr + 1, diff))
        if diff < 1.0e-5:
            break

    return x_opt


def calc_input(time):
    if time <= 7.5:  # wait at first
        v = 10.0
        yaw_rate = 0.0
    else:
        v = 10.0  # [m/s]
        yaw_rate = -0.8  # [rad/s]
    u = np.array([v, yaw_rate]).reshape(2, 1)
    return u


def observation(xTrue, xd, u, frame):

    # calc true state
    xTrue = motion_model(xTrue, u)

    # add noise to range observation
    z = np.zeros((3, 0))
    for i in range(len(frame)):
        dx = frame[i].x
        dy = frame[i].y
        dn = math.hypot(dx, dy) + np.random.randn() * Q_sim[0, 0]  # add noise
        anglen = pi_2_pi(math.atan2(dy, dx)) + \
            np.random.randn() * Q_sim[1, 1]  # add noise
        zi = np.array([dn, pi_2_pi(anglen), i]).reshape(3, 1)
        z = np.hstack((z, zi))

    # add noise to input
    ud1 = u[0, 0] + np.random.randn() * R_sim[0, 0]
    ud2 = u[1, 0] + np.random.randn() * R_sim[1, 1]  # + OFFSET_YAWRATE_NOISE
    ud = np.array([ud1, ud2]).reshape(2, 1)

    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud


def motion_model(x, u):
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT]])

    x = F @ x + B @ u

    return x


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def do_graph_slam(xEst, xTrue, xDR, hxEst, hxTrue, hxDR, u, motion, frame, hz, built_map, TRACK):
    
    xTrue, z, xDR, ud = observation(xTrue, xDR, u, frame)
    
    hz.append(z)

    x_opt = graph_based_slam(hxDR, hz)
    d_time = 0.0

    if show_animation:  
        np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)  
        plt.cla()
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(TRACK[:, 0], TRACK[:, 1], "*k")

        plt.plot(hxTrue[0, :].flatten(),
                    hxTrue[1, :].flatten(), "-b")
        plt.plot(hxDR[0, :].flatten(),
                    hxDR[1, :].flatten(), "-k")
        plt.plot(x_opt[0, :].flatten(),
                    x_opt[1, :].flatten(), "-r")
        plt.axis("equal")
        plt.grid(True)
        #plt.title("Time" + str(time)[0:5])
        plt.pause(1.0)
    return [[xEst], built_map]


def main():
    print("graph slamming...")

    time = 0.0

    sim_path = '../simulation/'
    # TRACK positions [x, y]
    trackfileloc = sim_path+'racetracks/track.csv'
    TRACK_DF = pd.read_csv(trackfileloc, header=None).drop(2, axis=1)
    TRACK = TRACK_DF.to_numpy()
    # n_cones = TRACK.shape[0]

    # Real perception input (frame by frame)
    frames_file = sim_path+'inputs/perception.pkl'
    with open(frames_file, 'rb') as lecturadeframes:
        lectura_frames = pickle.load(lecturadeframes)
    FRAMES_LIST = list(lectura_frames.values())
    FRAMES = [[Cone(c[0], c[1], c[2], c[3]) for c in f] for f in FRAMES_LIST]

#    STATES = [] # [[x1 y1 v1 yaw1],[x2 y2 v2 yaw2],..., [xn yn vn yawn]] # Where n is the number of frames
    motions_file = sim_path+'inputs/motions.pkl'
    with open(motions_file, 'rb') as lecturamotions:
        MOTIONS = pickle.load(lecturamotions)

    # State Vector [x y yaw v]'
    xEst = np.zeros((STATE_DIMS, 1))  # SLAM estimation
    xTrue = np.zeros((STATE_DIMS, 1))  # True state
    xDR = np.zeros((STATE_DIMS, 1))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue

    # while SLAMMING:
    dtime = 0.0
    init = False
    hz = []
    built_map = []
    for frame, motion in zip(FRAMES, MOTIONS):
        if not init:
            hxTrue = xTrue
            hxDR = xTrue
            init = True
        else:
            hxDR = np.hstack((hxDR, xDR))
            hxTrue = np.hstack((hxTrue, xTrue))
        
        time += 0.033
        dtime += 0.033

        u = calc_input(time)

        state, final_map = do_graph_slam(
                xEst, xTrue, xDR, hxEst, hxTrue, hxDR, u, motion, frame, hz, built_map, TRACK)


if __name__ == '__main__':
    main()