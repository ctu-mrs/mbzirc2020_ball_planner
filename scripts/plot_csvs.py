#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Point on a circle, defined by an angle
def circle_point_ang(orig, radius, angle):
    x = radius * np.cos(angle) + orig[0]
    y = radius * np.sin(angle) + orig[1]
    return np.array([x, y])

def sample_arc(orig, radius, start_ang, sample_dist, orientation = 1):
    angle = start_ang + orientation*(sample_dist/radius)
    pt = circle_point_ang(orig, radius, angle)
    return pt

def sample_arc_tangent(orig, radius, start_ang, sample_dist, orientation = 1):
    angle = start_ang + orientation*(sample_dist/radius)
    tangent = angle + orientation*np.pi/2.0
    return tangent

def sample_line(line_start, line_end, sample_dist):
    vec = line_end - line_start
    dist = np.linalg.norm(vec)
    dir = vec/dist
    pt = line_start + dir*sample_dist
    return pt

def sample_line_tangent(line_start, line_end, sample_dist):
    vec = line_end - line_start
    tangent = np.arctan2(vec[1], vec[0])
    return tangent

def ypr_to_R(yaw, pitch, roll):
    yaw_mat = np.matrix([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    pitch_mat = np.matrix([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    roll_mat = np.matrix([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    R = roll_mat*pitch_mat*yaw_mat
    return R

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

def normalize_angle(angle):
    out = np.fmod(angle, 2*np.pi);
    if out > np.pi:
      out -= 2*np.pi
    elif out < -np.pi:
      out += 2*np.pi
    return out

# from https://stackoverflow.com/questions/53033620/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr?rq=1
def ypr_to_quaternion(yaw, pitch, roll):
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        return [qw, qx, qy, qz]

def quaternion_to_rpy(w, x, y, z):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return roll, pitch, yaw

def load_data(ifname, delim=",", skip_header=False):
    flist = []
    rows = 0
    with open(ifname, 'r') as ifile:
        for line in ifile.readlines():
            if skip_header:
                skip_header = False
                continue
            sp = line.split(delim)
            for cell in sp:
                fnum = float(cell)
                flist.append(fnum)
            rows += 1
    ret = np.matrix(flist)
    ret = ret.reshape((rows, len(flist)/rows))
    return ret

def main():
    gts = load_data("data.csv", ',', True)
    ests = load_data("est.csv", ',', True)

    samples3D = gts[:, :3]
    ests3D = ests[:, :3]
    
    tangs = np.zeros((ests.shape[0], 3))
    it = 0
    for est in ests:
        yaw = est[0, 3]
        quat = est[0, -4:]
        [r, p, y] = quaternion_to_rpy(quat[0, 0], quat[0, 1], quat[0, 2], quat[0, 3])
        R = rpy_to_R(r, p, y)
        unit_vec = np.array([[np.cos(yaw)],[np.sin(yaw)],[0]])
        tan = (R*unit_vec).transpose()
        tangs[it, :] = tan
        it += 1
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    xs = np.array(samples3D[:, 0].transpose().flatten()).flatten()
    ys = np.array(samples3D[:, 1].transpose().flatten()).flatten()
    zs = np.array(samples3D[:, 2].transpose().flatten()).flatten()
    ax.plot(xs, ys, zs)
    # ax.plot(samples3D[:, 0].transpose().flatten(), samples3D[:, 1].transpose().flatten(), samples3D[:, 2].transpose().flatten(), 'x')
    xs = np.array(ests3D[:, 0].transpose().flatten()).flatten()
    ys = np.array(ests3D[:, 1].transpose().flatten()).flatten()
    zs = np.array(ests3D[:, 2].transpose().flatten()).flatten()
    ax.plot(xs, ys, zs)

    for it in range(0, len(tangs)):
        pt = ests3D[it, :]
        tan = tangs[it, :]
        pt2 = pt + tan
        ax.plot([pt[0,0], pt2[0,0]], [pt[0,1], pt2[0,1]], [pt[0,2], pt2[0,2]], 'r')

    ax.set_aspect('equal')
    plt.show()


if __name__ == "__main__":
    main()

