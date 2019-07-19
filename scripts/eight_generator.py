#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt

# Point on a circle, defined by an angle
def circle_point_ang(orig, radius, angle):
    x = radius * np.cos(angle) + orig[0]
    y = radius * np.sin(angle) + orig[1]
    return np.array([x, y])

def sample_arc(orig, radius, start_ang, sample_dist, orientation = 1):
    angle = start_ang + orientation*(sample_dist/radius)
    pt = circle_point_ang(orig, radius, angle)
    return pt

def sample_line(line_start, line_end, sample_dist):
    vec = line_end - line_start
    dist = np.linalg.norm(vec)
    dir = vec/dist
    pt = line_start + dir*sample_dist
    return pt

def rpy_to_R(roll, pitch, yaw):
    roll_mat = np.matrix([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    pitch_mat = np.matrix([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    yaw_mat = np.matrix([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    R = yaw_mat*pitch_mat*roll_mat
    return R

def save_data(data, ofname, col_names):
    with open(ofname, 'w') as of:
        txt = ",".join(col_names)
        of.write("{:s}\n".format(txt))
        rows = data.shape[0]
        for it in range(0, rows):
            row = data[it, :]
            txt = ",".join(map(str, row))
            of.write("{:s}\n".format(txt))


def main():
    # parameters of the pattern
    radius = 5.5 # metres
    width = 25   # metres
    lin_cutoff = 1.0 # metres, how much of the radius will be cut-off in favour of the linear segment

    # parameters of the sampling
    sample_dist = 0.5 # metres

    # parameters of transformation of the eight pattern
    pattern_rotation_rpy = [1.0, 2.0, 0.5] # radians
    pattern_translation  = [5.0, -3.0, 4.0] # metres

    ### ARCS
    # half of the angle of the circle sector, which is cut off by the linear segments
    arc_ang = np.arccos( (radius - lin_cutoff) / radius )

    # starting angle of the left arc
    l_arc_start_ang = arc_ang
    # ending angle of the left arc
    l_arc_end_ang = 2*np.pi - arc_ang
    # center of the left arc
    l_arc_orig = np.array([- width/2 + radius, 0])

    # starting angle of the right arc
    r_arc_start_ang = np.pi - arc_ang
    # ending angle of the right arc
    r_arc_end_ang = -np.pi + arc_ang
    # center of the right arc
    r_arc_orig = np.array([+ width/2 - radius, 0])

    ### LINES
    # starting point of the first line segment
    l_line_start = circle_point_ang(l_arc_orig, radius, l_arc_end_ang)
    # ending point of the first line segment
    l_line_end = circle_point_ang(r_arc_orig, radius, r_arc_start_ang)

    # starting point of the second line segment
    r_line_start = circle_point_ang(r_arc_orig, radius, r_arc_end_ang)
    # ending point of the second line segment
    r_line_end = circle_point_ang(l_arc_orig, radius, l_arc_start_ang)

    ### SAMPLING
    # perimeter of either of the arcs
    arc_len = radius * (2*np.pi - 2*arc_ang)
    # length of either of the lines
    line_len = np.linalg.norm(l_line_end - l_line_start)
    # total length of the eight pattern
    tot_len = 2*arc_len + 2*line_len

    # Recalculate the sample distance to obtain a full number of sample points
    n_pts = int(np.floor(tot_len/sample_dist))
    sample_dist = tot_len/n_pts

    samples = np.zeros((n_pts, 2))
    for it in range(0, n_pts):
        sector = 0 # sector 0 is the first (left) arc
        cur_dist = (it*sample_dist) % tot_len # current distance traveled along the eight-pattern
        cur_sector_dist = cur_dist # current distance traveled in the current sector
        if cur_dist > arc_len:
            sector = 1 # sector 1 is the first ("left") line
            cur_sector_dist = cur_dist - arc_len
        if cur_dist > arc_len + line_len:
            sector = 2 # sector 2 is the second (right) arc
            cur_sector_dist = cur_dist - (arc_len + line_len)
        if cur_dist > 2*arc_len + line_len:
            sector = 3 # sector 3 is the second ("right") line
            cur_sector_dist = cur_dist - (2*arc_len + line_len)

        cur_sample = None
        if sector == 0:
            cur_sample = sample_arc(l_arc_orig, radius, l_arc_start_ang, cur_sector_dist)
        elif sector == 1:
            cur_sample = sample_line(l_line_start, l_line_end, cur_sector_dist)
        elif sector == 2:
            cur_sample = sample_arc(r_arc_orig, radius, r_arc_start_ang, cur_sector_dist, -1)
        elif sector == 3:
            cur_sample = sample_line(r_line_start, r_line_end, cur_sector_dist)

        samples[it, :] = cur_sample
    
    samples3D = np.hstack([samples, np.zeros(n_pts, 1)])
    R = rpy_to_R(pattern_rotation_rpy[0], pattern_rotation_rpy[1], pattern_rotation_rpy[2])
    # TODO: rotate the samples, translate the samples

    save_data(samples, "data.csv", ["x", "y", "z"])

    plt.plot(samples[:, 0], samples[:, 1])
    plt.plot(samples[:, 0], samples[:, 1], 'x')
    plt.axis('equal')
    plt.show()

    # for it in range(0, len(samples)):
    #     plt.plot(samples[:it, 0], samples[:it, 1], 'x')
    #     plt.axis('equal')
    #     plt.xlim([-15, 15])
    #     plt.ylim([-10, 10])
    #     plt.show()


if __name__ == "__main__":
    main()
