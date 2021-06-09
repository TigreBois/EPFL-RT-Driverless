import numpy as np
from scipy.spatial import Delaunay
from numpy.linalg import norm
from math import pi
import math as mt
import matplotlib.pyplot as plt

max_iter = 40  # Number of middle points to be selected
point_step = 0.5  # Space between points in output in meters
origin = np.array([0, 0])  # origin = car position
rad_to_deg = 180 / pi


# Sort list a of points according to their norm
def sort_norm(a): return np.argsort(np.linalg.norm(a, axis=1))


# Filter out negative values on the y axis : cones behind the car
def filter_negative(yellow, blue):
    blue = blue[blue[:, 1] > 0]
    yellow = yellow[yellow[:, 1] > 0]
    return yellow, blue


# Compute the angle between vector v and w
def theta(v, w): return np.arccos(v.dot(w) / (np.linalg.norm(v) * np.linalg.norm(w)))


# Compute list of angles corresponding to the list of points ps
def get_angles(ps):
    v = np.subtract(ps[1:], ps[:-1])  # create vector list
    v = np.insert(v, 0, v[0], axis=0)
    allAngles = [theta(v[i - 1], v[i]) for i in range(1, len(v))]
    return np.array(allAngles) * rad_to_deg


# Return the middle points of a triangle
def mid_triangle(a): return [a[0] + a[1], a[1] + a[2], a[2] + a[0]]


# Invert middle point of triangle
def retrieve_cones(i1, i2, points):
    if i2 == 2:
        return np.array([points[i1][0], points[i1][2]])
    else:
        return np.array([points[i1][i2], points[i1][i2 + 1]])


# Recover the computed track
# return indexes of points in the tracks
def recover_track(inverse, duplicates_index, points_index):
    duplicates_index = inverse.reshape(-1)[duplicates_index]
    track = []
    for i in range(len(inverse)):
        mask = np.logical_not(np.isin(inverse[i], duplicates_index))
        a = (inverse[i])[mask]
        if len(a) == 1:
            index = np.where(inverse[i] == a[0])
            index = index[0][0]
            track.append(retrieve_cones(i, index, points_index))
    return np.array(track).reshape(-1)


def filter_angles(filtered, duplicates_index):
    angles = get_angles(filtered)
    i = 0
    while i < len(angles):
        while (angles[i] < 30) & (i < len(angles) - 1) & (i < max_iter):
            i = i + 1
        filtered = np.delete(filtered, i + 1, 0)
        duplicates_index = np.delete(duplicates_index, i)
        angles = get_angles(filtered)
    return duplicates_index


# Find midPoints that are in 2 different triangles
def duplicate_mid_points(mid_points):
    mid_points_index = np.arange(0, len(mid_points), 1, int)
    # Find the closest mid point to the origin
    norms = np.linalg.norm(mid_points, axis=1)
    closest = mid_points[np.argmin(norms)]
    closestIndex = np.argmin(norms)

    # Find midPoints that are in 2 different triangles
    u, indices, inverse = np.unique(mid_points, return_index=True, return_inverse=True, axis=0)
    inverse = indices[inverse].reshape(-1, 3)
    duplicates_index = np.delete(mid_points_index, indices)

    # Sort the duplicates according to the norm
    duplicates_points = mid_points[duplicates_index]
    duplicates_index = duplicates_index[sort_norm(duplicates_points)]

    # Add first point to which the car should go
    duplicates_index = np.insert(duplicates_index, 0, closestIndex)
    return duplicates_index, inverse


# Compute the middle path between the cones
def middle_path(yellow_cones, blue_cones):
    yellow_cones, blue_cones = filter_negative(yellow_cones, blue_cones)
    first_blue_index = len(yellow_cones)
    points = np.concatenate((yellow_cones, blue_cones))

    # Compute the Delaunay Triangulation formed by the cones
    # Compute the middle of all the edges of the triangulation
    tri = Delaunay(points)
    mid_points = (np.apply_along_axis(mid_triangle, 1, points[tri.simplices]) * 0.5).reshape(-1, 2)

    # Prepare index arrays
    points_index = np.arange(0, np.size(points[tri.simplices]) / 2, 1, int).reshape(-1, 3)
    duplicates_index, inverse = duplicate_mid_points(mid_points)

    # From the remaining points, filter points creating big angle turns
    filtered = np.insert(mid_points[duplicates_index], 0, origin, axis=0)  # add origin for computation
    track_index = filter_angles(filtered, duplicates_index)

    # Recover track boundaries corresponding to the computed path
    track = recover_track(inverse, track_index, points_index)
    track = (tri.simplices).reshape(-1)[track]
    track = np.unique(track)

    track_yellow = track[track < first_blue_index]
    track_blue = track[track >= first_blue_index] - first_blue_index
    path = mid_points[track_index]

    return path, track_yellow, track_blue


def local_to_global(path, car):
    angle = mt.atan2(car[0], car[1])
    new_path = [None for i in range(len(path))]
    for i in range(len(path)):
        current = path[i]
        x_glob = current[0] * mt.cos(angle) - current[1] * mt.sin(angle) + car[0]
        y_glob = current[0] * mt.sin(angle) + current[1] * mt.cos(angle) + car[1]
        new_path[i] = [x_glob, y_glob]

    return np.array(new_path)


