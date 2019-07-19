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

def main():
    # parameters of the pattern
    radius = 5.5 # metres
    width = 25   # metres
    lin_cutoff = 1.0 # metres, how much of the radius will be cut-off in favour of the linear segment

    # parameters of the sampling
    sample_dist = 0.25 # metres

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

    # l_arc_angs = np.arange(l_arc_start_ang, l_arc_end_ang, sample_ang)
    # r_arc_angs = np.arange(np.pi - arc_ang, -np.pi + arc_ang, -sample_ang)
    # l_arc_samps = sample_arc(l_arc_orig, radius, l_arc_angs)
    # r_arc_samps = sample_arc(r_arc_orig, radius, r_arc_angs)

    # l_line_samps = sample_line(l_line_start, l_line_end, sample_dist, False, True)
    # r_line_samps = sample_line(r_line_start, r_line_end, sample_dist, False, True)

    # plt.plot(l_arc_samps[:, 0], l_arc_samps[:, 1], 'x')
    # plt.plot(r_arc_samps[:, 0], r_arc_samps[:, 1], 'x')
    # plt.plot(l_line_samps[:, 0], l_line_samps[:, 1], 'x')
    # plt.plot(r_line_samps[:, 0], r_line_samps[:, 1], 'x')
    # plt.axis('equal')
    # plt.show()

    # all_samps = np.vstack([l_arc_samps, l_line_samps, r_arc_samps, r_line_samps])
    plt.plot(samples[:, 0], samples[:, 1])
    plt.plot(samples[:, 0], samples[:, 1], 'x')
    plt.axis('equal')
    # plt.xlim([0, 10])
    # plt.ylim([-4, 4])
    plt.show()

    # for it in range(0, len(all_samps)):
    #     plt.plot(all_samps[:it, 0], all_samps[:it, 1], 'x')
    #     plt.axis('equal')
    #     plt.xlim([0, 10])
    #     plt.ylim([-4, 4])
    #     plt.show()


if __name__ == "__main__":
    main()
