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

    # br = tf2_ros.TransformBroadcaster()
    # ts = geometry_msgs.msg.TransformStamped()
    br = tf.TransformBroadcaster()

    # find fiducial_id for markers
    marker_idx_0, marker_idx_1 = None, None
    for i, transfrom in enumerate(msg.transforms):
        if transfrom.fiducial_id == 0:
            marker_idx_0 = i
        elif transfrom.fiducial_id == 1:
            marker_idx_1 = i
    
    if marker_idx_0 == None or marker_idx_1 == None:
        rospy.logwarn("there is missing markers - id 0: {}, id 1: {}".format(marker_idx_0, marker_idx_1))
    else:
        marker_0 = msg.transforms[marker_idx_0].transform
        marker_1 = msg.transforms[marker_idx_1].transform  
        # map is average between marker 0 and marker 1
        cam_to_map_translation = (marker_0.translation.x/2 + marker_1.translation.x/2, 
                marker_0.translation.y/2 + marker_1.translation.y/2,
                marker_0.translation.z/2 + marker_1.translation.z/2)
        cam_to_map_quaternion = averageQuaternions(
            np.array([[marker_0.rotation.x, marker_0.rotation.y, 
                        marker_0.rotation.z, marker_0.rotation.w],
                    [marker_1.rotation.x, marker_1.rotation.y, 
                    marker_1.rotation.z, marker_1.rotation.w]]))
        cam_to_map_transform = t.concatenate_matrices(
                t.translation_matrix(cam_to_map_translation), 
                t.quaternion_matrix(cam_to_map_quaternion))
        map_to_cam_transform = t.inverse_matrix(cam_to_map_transform)

        # ts.header.stamp = rospy.Time.now()
        # ts.header.frame_id = '{}_rgb_camera_link'.format(camera_name)
        # ts.child_frame_id = "{}_base".format(camera_name)
        # ts.transform.translation.x = t.translation_from_matrix(cam_to_map_transform)[0]
        # ts.transform.translation.y = t.translation_from_matrix(cam_to_map_transform)[1]
        # ts.transform.translation.z = t.translation_from_matrix(cam_to_map_transform)[2]
        # ts.transform.rotation.x = t.quaternion_from_matrix(cam_to_map_transform)[0]
        # ts.transform.rotation.y = t.quaternion_from_matrix(cam_to_map_transform)[1]
        # ts.transform.rotation.z = t.quaternion_from_matrix(cam_to_map_transform)[2]
        # ts.transform.rotation.w = t.quaternion_from_matrix(cam_to_map_transform)[3]
        # br.sendTransform(ts)

        # fid0_to_map_translation = 
        br.sendTransform(
            t.translation_from_matrix(map_to_cam_transform),
            t.quaternion_from_matrix(map_to_cam_transform),
            rospy.Time.now(),
            '{}_camera_base'.format(camera_name),
            'map')


if __name__ == '__main__':

    rospy.init_node('camera_tf_broadcaster')
    camera_name = rospy.get_param('~camera')
    rospy.Subscriber('/{}/fiducial_transforms'.format(camera_name),
                    FiducialTransformArray,
                    handle_camera_pose,
                    camera_name)
    rospy.spin()