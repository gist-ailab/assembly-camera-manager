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
    camera_name_1 = rospy.get_param('~camera1')
    camera_name_2 = rospy.get_param('~camera2')

    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    rate = rospy.Rate(5.0)
    # camera_1_map = camera_2_map
    while not rospy.is_shutdown():
        now = rospy.Time()
        listener.waitForTransform(
                '{}_map'.format(camera_name_2), 
                '{}_camera_base'.format(camera_name_2),
                        now, rospy.Duration(1.0))
        (trans, rot) = listener.lookupTransform(
                    '{}_map'.format(camera_name_2), 
                    '{}_camera_base'.format(camera_name_2), 
                    now)

        br.sendTransform(
            trans,
            rot,
            rospy.Time.now(),
            '{}_camera_base'.format(camera_name_2),
            '{}_map'.format(camera_name_1)
        )
        rospy.loginfo_once("published tf: {}_map -> {}_camera_base".format(\
            camera_name_2, camera_name_1))

        rate.sleep()


