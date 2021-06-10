import copy
import math
import numpy as np
import itertools
from scipy.spatial.transform import Rotation as Rot

from slam.utils import normalizeAngle


# This algorithm can be made more optimal by upgrading to 
# a newer version uploaded to the PythonRobotics library

class GraphSLAM:
  def __init__(self, map, state_dims = 3, max_itr=20,c_sigma1=0.1,c_sigma2=0.1, c_sigma3=.0175):
      self.map = map
      self.hz = np.zeros((3, 0))
      self.hxDR = np.zeros((3, 0))

      self.state_dims = state_dims
      self.max_itr = max_itr

      self.c_sigma1 = c_sigma1
      self.c_sigma2 = c_sigma2
      self.c_sigma3 = c_sigma3

  def run(self, z, xDR, u):
      self.hz = np.hstack((self.hz, z))
      self.hxDR = np.hstack((self.hxDR, xDR))

      z_list = copy.deepcopy(self.hz)

      x_opt = copy.deepcopy(self.hxDR)
      nt = x_opt.shape[1]
      n = nt * self.state_dims

      for itr in range(self.max_itr):
          edges = self.calc_edges(x_opt, z_list)

          H = np.zeros((n, n))
          b = np.zeros((n, 1))

          for edge in edges:
              H, b = self.fill_H_and_b(H, b, edge)

          H[0:self.state_dims, 0:self.state_dims] += np.identity(self.state_dims)

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

  def cal_observation_sigma(self):
    sigma = np.zeros((3, 3))
    sigma[0, 0] = self.c_sigma1 ** 2
    sigma[1, 1] = self.c_sigma2 ** 2
    sigma[2, 2] = self.c_sigma3 ** 2

    return sigma


  def calc_rotational_matrix(self,angle):
    return Rot.from_euler('z', angle).as_matrix()


  def calc_edge(self,x1, y1, yaw1, x2, y2, yaw2, d1,
              angle1, d2, angle2, t1, t2):
    edge = Edge()

    tangle1 = normalizeAngle(yaw1 + angle1)
    tangle2 = normalizeAngle(yaw2 + angle2)
    tmp1 = d1 * math.cos(tangle1)
    tmp2 = d2 * math.cos(tangle2)
    tmp3 = d1 * math.sin(tangle1)
    tmp4 = d2 * math.sin(tangle2)

    edge.e[0, 0] = x2 - x1 - tmp1 + tmp2
    edge.e[1, 0] = y2 - y1 - tmp3 + tmp4
    edge.e[2, 0] = 0

    Rt1 = self.calc_rotational_matrix(tangle1)
    Rt2 = self.calc_rotational_matrix(tangle2)

    sig1 = self.cal_observation_sigma()
    sig2 = self.cal_observation_sigma()

    edge.omega = np.linalg.inv(Rt1 @ sig1 @ Rt1.T + Rt2 @ sig2 @ Rt2.T)

    edge.d1, edge.d2 = d1, d2
    edge.yaw1, edge.yaw2 = yaw1, yaw2
    edge.angle1, edge.angle2 = angle1, angle2
    edge.id1, edge.id2 = t1, t2

    return edge


  def calc_edges(self, x_list, z_list):
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

                    edge = self.calc_edge(x1, y1, yaw1, x2, y2, yaw2, d1,
                                      angle1, d2, angle2, t1, t2)

                    edges.append(edge)
                    cost += (edge.e.T @ edge.omega @ edge.e)[0, 0]

    print("cost:", cost, ",n_edge:", len(edges))
    return edges


  def calc_jacobian(self, edge):
    t1 = edge.yaw1 + edge.angle1
    A = np.array([[-1.0, 0, edge.d1 * math.sin(t1)],
                  [0, -1.0, -edge.d1 * math.cos(t1)],
                  [0, 0, 0]])

    t2 = edge.yaw2 + edge.angle2
    B = np.array([[1.0, 0, -edge.d2 * math.sin(t2)],
                  [0, 1.0, edge.d2 * math.cos(t2)],
                  [0, 0, 0]])

    return A, B


  def fill_H_and_b(self, H, b, edge):
    A, B = self.calc_jacobian(edge)

    id1 = edge.id1 * self.state_dims
    id2 = edge.id2 * self.state_dims

    H[id1:id1 + self.state_dims, id1:id1 + self.state_dims] += A.T @ edge.omega @ A
    H[id1:id1 + self.state_dims, id2:id2 + self.state_dims] += A.T @ edge.omega @ B
    H[id2:id2 + self.state_dims, id1:id1 + self.state_dims] += B.T @ edge.omega @ A
    H[id2:id2 + self.state_dims, id2:id2 + self.state_dims] += B.T @ edge.omega @ B

    b[id1:id1 + self.state_dims] += (A.T @ edge.omega @ edge.e)
    b[id2:id2 + self.state_dims] += (B.T @ edge.omega @ edge.e)

    return H, b

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


