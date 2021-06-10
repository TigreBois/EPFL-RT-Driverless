import math
import numpy as np
from slam.utils import normalizeAngle

def update_with_observation(particles, z, Q, ci, version=2):
    for iz in range(len(z[0, :])):
        coneid = int(z[2, iz])

        for ip in range(len(particles)):
            # new cone
            if abs(particles[ip].cones[coneid, 0]) <= ci:
                particles[ip] = add_new_cone(particles[ip], z[:, iz], Q)
            # known cone
            else:
                w = compute_weight(particles[ip], z[:, iz], Q)
                particles[ip].w *= w
                particles[ip] = update_cone(particles[ip], z[:, iz], Q)
                if version == 2:
                    particles[ip] = proposal_sampling(particles[ip], z[:, iz], Q)
    return particles


def compute_weight(particle, z, Q):
    cone_id = int(z[2])
    xf = np.array(particle.cones[cone_id, :]).reshape(2, 1)
    Pf = np.array(particle.conesP[2 * cone_id:2 * cone_id + 2])
    zp, Hv, Hf, Sf = compute_jacobians(particle, xf, Pf, Q)

    dz = z[0:2].reshape(2, 1) - zp
    dz[1, 0] = normalizeAngle(dz[1, 0])

    try:
        invS = np.linalg.inv(Sf)
    except np.linalg.linalg.LinAlgError:
        print("singuler")
        return 1.0

    num = math.exp(-0.5 * dz.T @ invS @ dz)
    den = 2.0 * math.pi * math.sqrt(np.linalg.det(Sf))
    w = num / den

    return w

def proposal_sampling(particle, z, Q):
    lm_id = int(z[2])
    xf = particle.lm[lm_id, :].reshape(2, 1)
    Pf = particle.lmP[2 * lm_id:2 * lm_id + 2]
    # State
    x = np.array([particle.x, particle.y, particle.yaw]).reshape(3, 1)
    P = particle.P
    zp, Hv, Hf, Sf = compute_jacobians(particle, xf, Pf, Q)

    Sfi = np.linalg.inv(Sf)
    dz = z[0:2].reshape(2, 1) - zp
    dz[1] = normalizeAngle(dz[1])

    Pi = np.linalg.inv(P)

    particle.P = np.linalg.inv(Hv.T @ Sfi @ Hv + Pi)  # proposal covariance
    x += particle.P @ Hv.T @ Sfi @ dz  # proposal mean

    particle.x = x[0, 0]
    particle.y = x[1, 0]
    particle.yaw = x[2, 0]

    return particle

def compute_jacobians(particle, xf, Pf, Q):
    dx = xf[0, 0] - particle.x
    dy = xf[1, 0] - particle.y
    d2 = dx**2 + dy**2
    d = math.sqrt(d2)

    zp = np.array(
        [d, normalizeAngle(math.atan2(dy, dx) - particle.yaw)]).reshape(2, 1)

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

    s = math.sin(normalizeAngle(particle.yaw + b))
    c = math.cos(normalizeAngle(particle.yaw + b))

    particle.cones[cone_id, 0] = particle.x + r * c
    particle.cones[cone_id, 1] = particle.y + r * s

    # Covariance
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
    dz[1, 0] = normalizeAngle(dz[1, 0])

    xf, Pf = update_KF_with_cholesky(xf, Pf, dz, Q, Hf)

    particle.cones[cone_id, :] = xf.T
    particle.conesP[2 * cone_id:2 * cone_id + 2, :] = Pf

    return particle