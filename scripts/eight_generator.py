#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt

def sample_arc(orig, radius, angs):
    pts = np.zeros((len(angs), 2))
    it = 0
    for ang in angs:
        x = radius * np.cos(ang) + orig[0]
        y = radius * np.sin(ang) + orig[1]
        pts[it, :] = np.array([x, y])
        it = it+1
    return pts

def sample_line(line_start, line_end, sample_dist, add_start = True, add_end = False):
    vec = line_end - line_start
    dist = np.linalg.norm(vec)
    step = vec/dist*sample_dist
    n_pts = int(np.floor(dist/sample_dist))
    pts = np.zeros((n_pts, 2))
    for it in range(0, n_pts):
        if it == 0 and not add_start:
            continue
        pt = line_start + it*step
        pts[it, :] = pt
    if not add_start:
        pts = pts[1:, :]
    if add_end:
        pts = np.vstack([pts, line_start + n_pts*step])
    return pts

def main():
    # parameters of the pattern
    radius = 5.5 # metres
    width = 25   # metres
    lin_cutoff = 1.0 # metres, how much of the radius will be cut-off in favour of the linear segment

    # parameters of the sampling
    sample_dist = 0.25 # metres

    arc_ang = np.arccos( (radius - lin_cutoff) / radius )
    sample_ang = sample_dist/radius
    l_arc_angs = np.arange(arc_ang, 2*np.pi - arc_ang, sample_ang)
    r_arc_angs = np.arange(np.pi - arc_ang, -np.pi + arc_ang, -sample_ang)

    l_arc_orig = np.array([- width/2 + radius, 0])
    r_arc_orig = np.array([+ width/2 - radius, 0])
    print(l_arc_orig)

    l_arc_samps = sample_arc(l_arc_orig, radius, l_arc_angs)
    r_arc_samps = sample_arc(r_arc_orig, radius, r_arc_angs)

    l_line_start = sample_arc(l_arc_orig, radius, [l_arc_angs[-1]])
    l_line_end = sample_arc(r_arc_orig, radius, [r_arc_angs[0]])
    r_line_start = sample_arc(r_arc_orig, radius, [r_arc_angs[-1]])
    r_line_end = sample_arc(l_arc_orig, radius, [l_arc_angs[0]])

    l_line_samps = sample_line(l_line_start, l_line_end, sample_dist, False, True)
    r_line_samps = sample_line(r_line_start, r_line_end, sample_dist, False, True)

    # plt.plot(l_arc_samps[:, 0], l_arc_samps[:, 1], 'x')
    # plt.plot(r_arc_samps[:, 0], r_arc_samps[:, 1], 'x')
    # plt.plot(l_line_samps[:, 0], l_line_samps[:, 1], 'x')
    # plt.plot(r_line_samps[:, 0], r_line_samps[:, 1], 'x')
    # plt.axis('equal')
    # plt.show()

    all_samps = np.vstack([l_arc_samps, l_line_samps, r_arc_samps, r_line_samps])
    plt.plot(all_samps[:, 0], all_samps[:, 1])
    plt.plot(all_samps[:, 0], all_samps[:, 1], 'x')
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
