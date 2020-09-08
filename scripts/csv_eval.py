#!/usr/bin/env python
import csv
import rospy
import tf2_py
import tf2_ros
import tf2_msgs.msg
import tf2_geometry_msgs
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped
from gazebo_msgs.msg import LinkStates
import numpy as np
import matplotlib.pyplot as plt

def load_csv_data(csv_fname):
    rospy.loginfo("Using CSV file {:s}".format(csv_fname))

    n_pos = sum(1 for line in open(csv_fname)) - 1
    data = np.zeros((n_pos, 4))
    it = 0
    with open(csv_fname, 'r') as fhandle:
        first_loaded = False
        csvreader = csv.reader(fhandle, delimiter=',')
        for row in csvreader:
            if not first_loaded:
                first_loaded = True
                continue
            data[it, :] = np.array([float(row[0]), float(row[1]), float(row[2]), int(row[3])])
            it += 1
    return data

def process_class(data, label, range):
    mean = np.mean(data)
    plt.figure()
    plt.hist(data, bins=50, range=range, density=True)
    # plt.plot([mean, mean], [0, 2])
    plt.title(label)
    print("Mean for {} is {} ({} total samples)".format(label, mean, len(data)))

def main():
    global tf_buffer, results
    rospy.init_node("mbzirc2020_eval")

    fname = rospy.get_param('~out_fname')
    data = load_csv_data(fname)
    dists = data[:, 1]
    errs = data[:, 2]
    cclasses = data[:, 3]
    max_dist = np.max(dists)
    print("max. det. dist: {}m".format(max_dist))
    print("max. hist. dist: {}m".format(10*np.ceil(max_dist/10)))

    process_class(dists[cclasses[:] == 0], "Class C distances", [0, 10*np.ceil(max_dist/10)])
    process_class(dists[cclasses[:] == 1], "Class B distances", [0, 10*np.ceil(max_dist/10)])
    process_class(dists[cclasses[:] == 2], "Class A distances", [0, 10*np.ceil(max_dist/10)])
    plt.show()

    # process_class(errs[cclasses[:] == 0], "Class C errors", [0,2])
    # process_class(errs[cclasses[:] == 1], "Class B errors", [0,2])
    # process_class(errs[cclasses[:] == 2], "Class A errors", [0,2])
    # plt.show()

if __name__ == '__main__':
    main()
