import sys
import math
import rospy
import copy
import tf.transformations
import tf2_ros
import csv
from shape_msgs.msg import SolidPrimitive
#T
# messages
import geometry_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Bool

class ArucoCheck:
    def get_all_tfs(self):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        start = rospy.get_time()
        now = rospy.get_time()
        period = 1
        tfs = None
        while now - start < period:
            try:
                tfs = tfBuffer._getFrameStrings()
                now = rospy.get_time()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                now = rospy.get_time()
                rospy.sleep(0.2)
                continue
        return tfs
    # Gets the pose of given marker_id
    def get_marker_pose(self, marker_id):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        start = rospy.get_time()
        now = rospy.get_time()
        period = 1
        marker_frame = 'fiducial_' + str(marker_id)
        transform = None
        while now - start < period:
            try:
                # getting the transform between camera frame(as defined in camera_tf.launch) and aruco
                transform = tfBuffer.lookup_transform('cgo3_camera_optical_frame',
                                                      marker_frame,
                                                      rospy.Time(0))
                now = rospy.get_time()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                now = rospy.get_time()
                rospy.sleep(0.2)
                continue
        return transform
    
    # Check if required Aruco has been detected
    def check_for_aruco(self, ignore=[]):
        rospy.loginfo("Extracting marker poses....")
        tfs = self.get_all_tfs()
        marker_frames = [
            marker_frame for marker_frame in tfs if "fiducial_" in marker_frame
        ]
        rospy.loginfo("Detected markers %s", marker_frames)
        for frame in marker_frames:
            marker_id = int(frame.replace('fiducial_', '', 1))
            marker_pose = self.get_marker_pose(marker_id)

            #check if target Aruco has been detected, marker 0
            if marker_id==0:
                print("Target marker has been detected")
                return marker_pose #returns an array of x, y, z, r, p, y of aruco tag wrt camera frame
        # if not found,
        return -1 
