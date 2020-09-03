#!/usr/bin/env python

import rospy
import roslib
from fiducial_msgs.msg import FiducialTransformArray
import tf
from tf import transformations as t
import numpy as np
import numpy.matlib as npm
import tf2_ros
import tf_conversions
import geometry_msgs.msg

# https://answers.ros.org/question/322317/combine-two-parent-child-transformations-for-common-link/
if __name__ == '__main__':


    rospy.init_node('map_broadcaster')
    camera_name = rospy.get_param('~camera_name')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(5.0)

    while not rospy.is_shutdown():
        rospy.wait_for_service('/{}/extrinsic_calibration'.format(camera_name))
        try:
            transformstamped = tfBuffer.lookup_transform(
                '{}_camera_fid_0'.format(camera_name), 
                '{}_camera_base'.format(camera_name), rospy.Time())

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        br = tf2_ros.StaticTransformBroadcaster()
        static_tf = geometry_msgs.msg.TransformStamped()
        static_tf.header.stamp = rospy.Time.now()
        static_tf.header.frame_id = 'azure1_camera_fid_0'
        static_tf.child_frame_id = '{}_camera_base'.format(camera_name)
        static_tf.transform = transformstamped.transform
        br.sendTransform(static_tf)

        rospy.loginfo_once("published static tf: azure1_camera_fid_0 -> {}_camera_base".format(camera_name))

