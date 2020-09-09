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

def put_to_file(data, fname):
    with open(fname, 'w') as ofhandle:
        ofhandle.write("dist, err, recall, pdet, pcclassA, pcclassB, pcclassC\n")
        for it in range(0, len(data)):
            ofhandle.write("{:f},{:f},{:f},{:f},{:f},{:f},{:f}\n".format(data[it][0], data[it][1], data[it][2], data[it][3], data[it][4], data[it][5], data[it][6]))

def process_class(data, label, hrange):
    mean = np.mean(data)
    plt.figure()
    plt.hist(data, bins=50, range=hrange, density=True)
    # plt.plot([mean, mean], [0, 2])
    plt.title(label)
    print("Mean for {} is {} ({} total samples)".format(label, mean, len(data)))

# #{ 

def process_dist(ofname, dists, errs, cclasses, hrange):
    bin_w = 1.0 # m

    mean_errs = list()
    recalls = list()
    bin_ctrs = list()
    min_dist = -bin_w/2.0
    pcclassA = list()
    pcclassB = list()
    pcclassC = list()
    for max_dist in np.arange(bin_w/2.0, hrange[1]+3*bin_w/2.0, bin_w):
        idxs = np.logical_and(dists > min_dist, dists <= max_dist)
        cur_errs = errs[idxs]
        cur_total = float(len(cur_errs))
        cur_detected = float(len(cur_errs[np.isfinite(cur_errs)]))
        cur_recall = np.nan
        cur_mean_err = np.nan
        if cur_total > 0:
            cur_recall = cur_detected/cur_total
            cur_mean_err = np.mean(cur_errs[np.isfinite(cur_errs)])
        cur_bin_ctr = (max_dist + min_dist)/2.0

        mean_errs.append(cur_mean_err)
        recalls.append(cur_recall)
        bin_ctrs.append(cur_bin_ctr)
        min_dist = max_dist

        cur_cclasses = cclasses[idxs]
        cur_cclassA = len(cur_cclasses[cur_cclasses == 2])
        cur_cclassB = len(cur_cclasses[cur_cclasses == 1])
        cur_cclassC = len(cur_cclasses[cur_cclasses == 0])
        cur_pcclassA = 0
        cur_pcclassB = 0
        cur_pcclassC = 0
        if cur_detected > 0:
            cur_pcclassA = cur_cclassA/cur_total
            cur_pcclassB = cur_cclassB/cur_total
            cur_pcclassC = cur_cclassC/cur_total
        pcclassA.append(cur_pcclassA)
        pcclassB.append(cur_pcclassB)
        pcclassC.append(cur_pcclassC)

    # calculation of the theoretical detection probability
    # pdists = np.arange(1, hrange[1], 0.1)
    pdists = np.array(bin_ctrs)
    vang_res = (np.pi/180 * 45/64)
    hang_res = (2*np.pi / 2048)
    n_pts_min = 1
    tgtw = 0.17
    tgth = 0.08 + 0.15 # mav height and ball height
    pdet_vert = np.minimum( np.arctan(tgth/(pdists-tgtw/2))/(vang_res*n_pts_min), 1.0);
    pdet_hori = np.minimum( np.arctan(tgtw/pdists)/(hang_res*3), 1.0);
    pdet = pdet_vert * pdet_hori
    pdet[0] = 1.0

    recalls[0] = 0.0
    # bin_ctrs.insert(0, 0.0) # first element is zero
    # mean_errs.insert(0, np.nan)
    # recalls.insert(0, 0.0)
    # pdet = np.concatenate(([1.0], pdet))
    # pcclassA.insert(0, 0.0)
    # pcclassB.insert(0, 0.0)
    # pcclassC.insert(0, 0.0)

    plt.figure()
    plt.plot(bin_ctrs, mean_errs, ".", bin_ctrs, recalls, ".", bin_ctrs, pdet)
    # plt.step(bin_ctrs, pcclassA, bin_ctrs, pcclassB, bin_ctrs, pcclassC, where="mid")
    plt.legend(["error", "recall", "theoretical recall"])
    # plt.plot(bin_ctrs, recalls, pdists, pdet)
    # plt.legend(["recall", "theoretical recall"])
    plt.xlabel("target-sensor distance (m)")

    plt.figure()
    plt.plot(bin_ctrs, pcclassA, bin_ctrs, pcclassB, bin_ctrs, pcclassC)
    # plt.plot(bin_ctrs, pcclassA, ".", bin_ctrs, pcclassB, ".", bin_ctrs, pcclassC, ".")
    # plt.bar(np.array(bin_ctrs)-1/3.0, pcclassA, width=1/3.0)
    # plt.bar(bin_ctrs, pcclassB, width=1/3.0)
    # plt.bar(np.array(bin_ctrs)+1/3.0, pcclassC, width=1/3.0)
    plt.legend(["class A", "class B", "class C"])
    plt.ylabel("rate of class")
    plt.xlabel("target-sensor distance (m)")
    # plt.hist(data, bins=bin_edges)
    # plt.plot([mean, mean], [0, 2])
    # print("Mean for {} is {} ({} total samples)".format(label, mean, len(data)))

    print(len(bin_ctrs))
    print(len(mean_errs))
    print(len(recalls))
    print(len(pdet))
    print(len(pcclassA))
    print(len(pcclassB))
    print(len(pcclassC))
    result = np.column_stack([bin_ctrs, mean_errs, recalls, pdet, pcclassA, pcclassB, pcclassC])
    put_to_file(result, ofname)

# #} end of 

def main():
    global tf_buffer, results
    rospy.init_node("mbzirc2020_eval")

    ifname = rospy.get_param('~in_fname')
    ofname = rospy.get_param('~out_fname')
    data = load_csv_data(ifname)
    dists = data[:, 1]
    errs = data[:, 2]
    cclasses = data[:, 3]
    max_dist = np.max(dists)
    print("max. det. dist: {}m".format(max_dist))
    print("max. hist. dist: {}m".format(10*np.ceil(max_dist/10)))

    process_dist(ofname, dists, errs, cclasses, [0, 10*np.ceil(max_dist/10)])
    # plt.show()

    # process_class(dists[cclasses[:] == 0], "Class C distances", [0, 10*np.ceil(max_dist/10)])
    # process_class(dists[cclasses[:] == 1], "Class B distances", [0, 10*np.ceil(max_dist/10)])
    # process_class(dists[cclasses[:] == 2], "Class A distances", [0, 10*np.ceil(max_dist/10)])
    # plt.figure()
    # plt.hist(dists)
    # plt.show()

    # process_class(errs[cclasses[:] == 0], "Class C errors", [0,2])
    # process_class(errs[cclasses[:] == 1], "Class B errors", [0,2])
    # process_class(errs[cclasses[:] == 2], "Class A errors", [0,2])
    # plt.show()

if __name__ == '__main__':
    main()
