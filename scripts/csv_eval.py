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
    data = np.zeros((n_pos, 13))
    it = 0
    with open(csv_fname, 'r') as fhandle:
        first_loaded = False
        csvreader = csv.reader(fhandle, delimiter=',')
        for row in csvreader:
            if not first_loaded:
                first_loaded = True
                continue
#     0, 1, 2, 3, 4, 5, 6, 7, 8, 9,  10, 11,    12
# stamp,mx,my,mz,dx,dy,dz,bx,by,bz,dist,err,cclass
            stamp = float(row[0])
            mpos = np.array([float(row[1]), float(row[2]), float(row[3])])
            dpos = np.array([float(row[4]), float(row[5]), float(row[6])])
            bpos = np.array([float(row[7]), float(row[8]), float(row[9])])
            dist = float(row[10])
            err = float(row[11])
            cclass = float(row[12])
            data[it, :] = np.array([stamp, mpos[0], mpos[1], mpos[2], dpos[0], dpos[1], dpos[2], bpos[0], bpos[1], bpos[2], dist, err, cclass])
            it += 1
    return data

def put_to_file(labels, data, fname):
    fmt_str = ",".join((["{:f}"]*len(data[0]))) + "\n"
    with open(fname, 'w') as ofhandle:
        ofhandle.write("{:s}\n".format(",".join(labels)))
        for it in range(0, len(data)):
            ofhandle.write(fmt_str.format(*data[it]))

# #{ 

def process_class(ofname, data, cclasses, hrange):
    # mean = np.mean(data)
    # std = np.std(data)
    # print("Mean for {} is {}, std is {} ({} total samples)".format(label, mean, std, len(data)))

    bin_w = 0.05 # m

    bin_ctrs = list()
    cclassA = list()
    cclassB = list()
    cclassC = list()
    acclassA = list()
    acclassB = list()
    acclassC = list()

    bin_ctrs.append(0.0)
    cclassA.append(0.0)
    cclassB.append(0.0)
    cclassC.append(0.0)
    acclassA.append(0.0)
    acclassB.append(0.0)
    acclassC.append(0.0)

    min_err = 0
    for max_err in np.arange(bin_w, hrange[1]+3*bin_w, bin_w):
        idxs = np.logical_and(data > min_err, data <= max_err)

        cur_cclasses = cclasses[idxs]
        cur_cclassA = float(len(cur_cclasses[cur_cclasses == 2]))
        cur_cclassB = float(len(cur_cclasses[cur_cclasses == 1]))
        cur_cclassC = float(len(cur_cclasses[cur_cclasses == 0]))
        cclassA.append(cur_cclassA)
        cclassB.append(cur_cclassB)
        cclassC.append(cur_cclassC)

        acclassA.append(acclassA[-1] + cur_cclassA)
        acclassB.append(acclassB[-1] + cur_cclassB)
        acclassC.append(acclassC[-1] + cur_cclassC)

        cur_bin_ctr = (max_err + min_err)/2.0
        bin_ctrs.append(cur_bin_ctr)
        min_err = max_err

    pcclassA = cclassA/np.sum(cclassA)
    pcclassB = cclassB/np.sum(cclassB)
    pcclassC = cclassC/np.sum(cclassC)

    acclassA = np.array(acclassA)/acclassA[-1]
    acclassB = np.array(acclassB)/acclassB[-1]
    acclassC = np.array(acclassC)/acclassC[-1]

    plt.figure()
    plt.plot(bin_ctrs, pcclassA, "-.", bin_ctrs, pcclassB, "-.", bin_ctrs, pcclassC, "-.")
    plt.legend(["class A", "class B", "class C"])
    plt.ylabel("rate of class")
    plt.xlabel("localization error (m)")

    result = np.column_stack([bin_ctrs, pcclassA, pcclassB, pcclassC, acclassA, acclassB, acclassC])
    labels = ["err,pcclassA,pcclassB,pcclassC,acclassA,acclassB,acclassC"]
    put_to_file(labels, result, ofname)

# #} end of 

# #{ 

def process_dist(ofname, mpos, dpos, bpos, dists, errs, cclasses, hrange):
    bin_w = 1.0 # m
    vangle = 33.2/180.0*np.pi

    mean_errs = list()
    recalls = list()
    bin_ctrs = list()

    pcclassA = list()
    pcclassB = list()
    pcclassC = list()

    min_dist = -bin_w/2.0
    for max_dist in np.arange(bin_w/2.0, hrange[1]+3*bin_w/2.0, bin_w):
        idxs = np.logical_and(dists > min_dist, dists <= max_dist)
        cur_errs = errs[idxs]

        cur_mposs = mpos[idxs, :]
        cur_dposs = dpos[idxs, :]
        cur_bposs = bpos[idxs, :]

        mdvec = cur_dposs-cur_mposs
        mdang = np.abs(np.arctan2(mdvec[:, 2], np.linalg.norm(mdvec[:, 0:2], axis=1)))
        dpos_detectable = mdang < vangle/2.0

        mbvec = cur_bposs-cur_mposs
        mbang = np.abs(np.arctan2(mbvec[:, 2], np.linalg.norm(mbvec[:, 0:2], axis=1)))
        bpos_detectable = mbang < vangle/2.0
        # cur_valid = np.logical_or(dpos_detectable, bpos_detectable).sum()
        cur_valid = float(dpos_detectable.sum())

        # if not dpos_detectable.all() or not bpos_detectable.all():
        #     print("JOOO")

        cur_detected = float(len(cur_errs[np.isfinite(cur_errs)]))
        cur_recall = 0.0
        cur_mean_err = np.nan
        if cur_valid > 0:
            cur_recall = np.minimum(cur_detected/cur_valid, 1.0)
            cur_mean_err = np.mean(cur_errs[np.isfinite(cur_errs)])
        cur_bin_ctr = (max_dist + min_dist)/2.0

        mean_errs.append(cur_mean_err)
        recalls.append(cur_recall)
        bin_ctrs.append(cur_bin_ctr)
        min_dist = max_dist

        cur_cclasses = cclasses[idxs]
        cur_cclassA = (cur_cclasses == 2).sum()
        cur_cclassB = (cur_cclasses == 1).sum()
        cur_cclassC = (cur_cclasses == 0).sum()
        cur_cclassD = (cur_cclasses == 4).sum()
        cur_cclassTot = cur_cclassA + cur_cclassB + cur_cclassC + cur_cclassD
        print("{} + {} + {} + {} = {} (should be {})".format(cur_cclassA, cur_cclassB, cur_cclassC, cur_cclassD, cur_cclassTot, len(cur_cclasses)))
        cur_pcclassA = 0
        cur_pcclassB = 0
        cur_pcclassC = 0
        if cur_valid > 0:
            cur_pcclassA = np.minimum(cur_cclassA/cur_valid, 1.0)
            cur_pcclassB = np.minimum(cur_cclassB/cur_valid, 1.0)
            cur_pcclassC = np.minimum(cur_cclassC/cur_valid, 1.0)
        pcclassA.append(cur_pcclassA)
        pcclassB.append(cur_pcclassB)
        pcclassC.append(cur_pcclassC)

    # calculation of the theoretical detection probability
    # pdists = np.arange(1, hrange[1], 0.1)
    pdists = np.array(bin_ctrs)
    vang_res = (vangle/32)
    hang_res = (2*np.pi / 2048)
    n_vpts_min = 1
    n_hpts_min = 3
    tgtw = 0.15
    tgth = 0.10 + 0.15 # mav height and ball height
    # pdet_vert = np.zeros((len(pdists), n_vpts_min+1))
    # for it in range(1, n_vpts_min+1):
    #     pdet_vert[:, it] = np.minimum( np.arctan(tgth/(pdists-tgtw/2))/(vang_res*it), 1.0)
    # for it in np.arange(n_vpts_min, 0, -1):
    #     for it2 in range(0, it):
    #         pdet_vert[:, it] -= pdet_vert[:, it2]

    # pdet_hori = np.zeros((len(pdists), n_hpts_min+1))
    # for it in range(1, n_hpts_min+1):
    #     pdet_hori[:, it] = np.minimum( np.arctan(tgtw/pdists)/(hang_res*it), 1.0)
    # for it in np.arange(n_hpts_min, 0, -1):
    #     for it2 in range(0, it):
    #         pdet_hori[:, it] -= pdet_hori[:, it2]
    # pdet = np.minimum( \
    #     pdet_vert[:, 1] * pdet_hori[:, 3] \
    #     + pdet_vert[:, 2] * pdet_hori[:, 2] \
    #     + pdet_vert[:, 2] * pdet_hori[:, 3] \
    #     + pdet_vert[:, 3] * pdet_hori[:, 3] \
    #     + pdet_vert[:, 3] * pdet_hori[:, 2] \
    #     + pdet_vert[:, 3] * pdet_hori[:, 1] \
    #                   , 1.0
    #                   )
    # print(pdet_vert)
    # print(pdet_hori)
    # pdet = \
    #     pdet_vert[:, 1] * pdet_hori[:, 3] \
    #     + pdet_vert[:, 2] * pdet_hori[:, 2] \
    #     + pdet_vert[:, 2] * pdet_hori[:, 3] \
    #     + pdet_vert[:, 3] * pdet_hori[:, 3] \
    #     + pdet_vert[:, 3] * pdet_hori[:, 2] \
    #     + pdet_vert[:, 3] * pdet_hori[:, 1] \

    pray = 0.8
    pdet_vert = np.minimum( pray*np.arctan(tgth/(pdists-tgtw/2))/(vang_res*n_vpts_min), 1.0)
    pdet_hori = np.minimum( pray*np.arctan(tgtw/pdists)/(hang_res*n_hpts_min), 1.0)
    pdet = pdet_vert * pdet_hori

    # pdet = pdet_hori
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
    labels = ["dist,err,recall,pdet,pcclassA,pcclassB,pcclassC"]
    put_to_file(labels,result, ofname)

# #} end of 

def main():
    global tf_buffer, results
    rospy.init_node("mbzirc2020_eval")

    ifname = rospy.get_param('~in_fname')
    stats_ofname = rospy.get_param('~stats_out_fname')
    class_ofname = rospy.get_param('~class_out_fname')
    data = load_csv_data(ifname)
#     0, 1, 2, 3, 4, 5, 6, 7, 8, 9,  10, 11,    12
# stamp,mx,my,mz,dx,dy,dz,bx,by,bz,dist,err,cclass
    mpos = data[:, 1:4]
    dpos = data[:, 4:7]
    bpos = data[:, 7:10]
    dists = data[:, 10]
    errs = data[:, 11]
    cclasses = data[:, 12]
    max_dist = np.max(dists)
    print("mean error: {}m, std. error: {}m".format(np.mean(errs[np.isfinite(errs)]), np.std(errs[np.isfinite(errs)])))
    print("max. det. dist: {}m".format(max_dist))
    # print("max. hist. dist: {}m".format())

    process_dist(stats_ofname, mpos, dpos, bpos, dists, errs, cclasses, [0, 90])
    plt.show()

    # process_class(dists[cclasses[:] == 0], "Class C distances", [0, 10*np.ceil(max_dist/10)])
    # process_class(dists[cclasses[:] == 1], "Class B distances", [0, 10*np.ceil(max_dist/10)])
    # process_class(dists[cclasses[:] == 2], "Class A distances", [0, 10*np.ceil(max_dist/10)])
    # plt.figure()
    # plt.hist(dists)
    # plt.show()

    process_class(class_ofname, errs, cclasses, [0,2])
    # plt.show()

if __name__ == '__main__':
    main()
