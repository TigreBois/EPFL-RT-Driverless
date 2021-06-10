import numpy as np
from slam.utils import normalizeAngle

def normalize_weight(particles):

    sumw = sum([p.w for p in particles])

    try:
        for i in range(len(particles)):
            particles[i].w /= sumw
    except ZeroDivisionError:
        for i in range(len(particles)):
            particles[i].w = 1.0 / len(particles)

        return particles

    return particles


def resampling(particles, n_resampled):
    """
    low variance re-sampling
    """

    n_particles = len(particles)
    particles = normalize_weight(particles)

    pw = []
    for i in range(n_particles):
        pw.append(particles[i].w)

    pw = np.array(pw)

    Neff = 1.0 / (pw @ pw.T)  # Effective particle number

    if Neff < n_resampled:  # resampling
        wcum = np.cumsum(pw)
        base = np.cumsum(pw * 0.0 + 1 / n_particles) - 1 / n_particles
        resampleid = base + np.random.rand(base.shape[0]) / n_particles

        inds = []
        ind = 0
        for ip in range(n_particles):
            while ((ind < wcum.shape[0] - 1) and (resampleid[ip] > wcum[ind])):
                ind += 1
            inds.append(ind)

        tparticles = particles[:]
        for i in range(len(inds)):
            particles[i].x = tparticles[inds[i]].x
            particles[i].y = tparticles[inds[i]].y
            particles[i].yaw = tparticles[inds[i]].yaw
            particles[i].w = 1.0 / n_particles

    return particles

def get_final_state(particles, state_dims=3):
    xEst = np.zeros((state_dims, 1))

    particles = normalize_weight(particles)

    for i in range(len(particles)):
        xEst[0, 0] += particles[i].w * particles[i].x
        xEst[1, 0] += particles[i].w * particles[i].y
        xEst[2, 0] += particles[i].w * particles[i].yaw

    xEst[2, 0] = normalizeAngle(xEst[2, 0])

    return xEst