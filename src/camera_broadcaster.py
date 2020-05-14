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

# reference:https://github.com/christophhagen/averaging-quaternions
def averageQuaternions(Q):
    # Number of quaternions to average
    M = Q.shape[0]
    A = npm.zeros(shape=(4,4))

    for i in range(0,M):
        q = Q[i,:]
        # multiply q with its transposed version q' and add A
        A = np.outer(q,q) + A

    # scale
    A = (1.0/M)*A
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    return np.real(eigenVectors[:,0].A1)


def handle_camera_pose(msg, camera_name):

    br = tf.TransformBroadcaster()

    # find fiducial_id for markers
    marker_idx_0, marker_idx_1 = None, None
    for i, Fidtransform in enumerate(msg.transforms): 
        # publish all detected fiducial as tf
        trans = Fidtransform.transform.translation
        rot = Fidtransform.transform.rotation
        br.sendTransform(
            (trans.x, trans.y, trans.z),
            (rot.x, rot.y, rot.z, rot.w),
            rospy.Time.now(),
            '{}_fid_{}'.format(camera_name, Fidtransform.fiducial_id),
            msg.header.frame_id,
            )
        rospy.loginfo_once("published tf: {} -> {}_fid_{}".format(\
            msg.header.frame_id,camera_name, Fidtransform.fiducial_id ))
        if Fidtransform.fiducial_id == 0:
            marker_idx_0 = i
        elif Fidtransform.fiducial_id == 1:
            marker_idx_1 = i        

    if marker_idx_0 == None or marker_idx_1 == None:
        rospy.logwarn("there is missing markers - \
            id 0: {}, id 1: {}".format(marker_idx_0, marker_idx_1))
    else:
        trans_marker0 = msg.transforms[marker_idx_0].transform.translation
        rot_marker0 = msg.transforms[marker_idx_0].transform.rotation
        trans_marker1 = msg.transforms[marker_idx_1].transform.translation
        rot_marker1 = msg.transforms[marker_idx_1].transform.rotation
        # map is average between marker 0 and marker 1
        br.sendTransform(
            (trans_marker0.x/2 + trans_marker1.x/2, 
                trans_marker0.y/2 + trans_marker1.y/2,
                trans_marker0.z/2 + trans_marker1.z/2),
            averageQuaternions(
                np.array([[rot_marker0.x, rot_marker0.y, 
                            rot_marker0.z, rot_marker0.w],
                        [rot_marker1.x, rot_marker1.y, 
                            rot_marker1.z, rot_marker1.w]])),
            rospy.Time.now(),
            '{}_map'.format(camera_name),
            msg.header.frame_id,)
        rospy.loginfo_once("published tf: {} -> {}_map".format(msg.header.frame_id, camera_name))



if __name__ == '__main__':

    rospy.init_node('camera_broadcaster')
    camera_name = rospy.get_param('~camera')
    rospy.Subscriber('/{}/fiducial_transforms'.format(camera_name),
                    FiducialTransformArray,
                    handle_camera_pose,
                    camera_name)
    rospy.spin()