#!/usr/bin/env python
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

tf_buffer = None
global_frame_id = "common_origin"

bllpos_msg = None
mavpos_msg = None
dropos_msg = None

results = list()

def det_callback(msg):
    global tf_buffer, bllpos_msg, mavpos_msg, dropos_msg, results

    if bllpos_msg is None:
        rospy.logwarn("no ballpose")
        return
    trans = None
    try:
        trans = tf_buffer.lookup_transform(global_frame_id, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.08))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("no transform")
        return
    gen = pc2.read_points(msg)
    pt = None
    for i in gen:
        pt = i
        break

    bllpos = np.array([bllpos_msg.x, bllpos_msg.y, bllpos_msg.z])
    mavpos = np.array([mavpos_msg.x, mavpos_msg.y, mavpos_msg.z])
    dropos = np.array([dropos_msg.x, dropos_msg.y, dropos_msg.z])
    dist = np.linalg.norm(mavpos - bllpos)
    err = np.nan
    cclass = 4
    if pt is not None: # not an empty point - got a detection
        cclass = pt[3]
        pts = PointStamped()
        pts.point.x = pt[0]
        pts.point.y = pt[1]
        pts.point.z = pt[2]
        detpos_msg = tf2_geometry_msgs.do_transform_point(pts, trans).point

        detpos = np.array([detpos_msg.x, detpos_msg.y, detpos_msg.z])
        err = np.linalg.norm(detpos - bllpos)

    datum = (msg.header.stamp.to_sec(), mavpos[0], mavpos[1], mavpos[2], dropos[0], dropos[1], dropos[2], bllpos[0], bllpos[1], bllpos[2], dist, err, cclass)
    print(datum)
    results.append(datum)

def ball_callback(msg):
    global bllpos_msg
    it = 0
    for name in msg.name:
        if name == "uav60::ball_holder_ball":
            break
        it += 1
    if it < len(msg.pose):
        bllpos_msg = msg.pose[it].position

def mav_callback(msg):
    global mavpos_msg
    mavpos_msg = msg.pose.pose.position

def drone_callback(msg):
    global dropos_msg
    dropos_msg = msg.pose.pose.position

def put_to_file(labels, data, fname):
    fmt_str = ",".join((["{:f}"]*len(data[0]))) + "\n"
    with open(fname, 'w') as ofhandle:
        ofhandle.write("{:s}\n".format(",".join(labels)))
        for it in range(0, len(data)):
            ofhandle.write(fmt_str.format(*data[it]))
    
def main():
    global tf_buffer, results
    rospy.init_node("mbzirc2020_eval")

    fname = rospy.get_param('~out_fname')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.Subscriber("/uav62/uav_detection/detections_pc", PointCloud2, det_callback)
    rospy.Subscriber("/gazebo/link_states", LinkStates, ball_callback)
    rospy.Subscriber("/uav62/ground_truth_pose", PoseWithCovarianceStamped, mav_callback)
    rospy.Subscriber("/uav60/ground_truth_pose", PoseWithCovarianceStamped, drone_callback)

    rospy.spin()
    labels = ["stamp,mx,my,mz,dx,dy,dz,bx,by,bz,dist,err,cclass"]
    put_to_file(labels, results, fname)
    # put_to_file(results, fname)

if __name__ == '__main__':
    main()

