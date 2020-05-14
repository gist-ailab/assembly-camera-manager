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
    listener = tf.TransformListener()

    # find fiducial_id for markers
    marker_idx_0, marker_idx_1 = None, None
    for i, Fidtransform in enumerate(msg.transforms):
        if Fidtransform.fiducial_id == 0:
            marker_idx_0 = i
        elif Fidtransform.fiducial_id == 1:
            marker_idx_1 = i
        # publish all detected fiducial as tf
        # trans = Fidtransform.transform.translation
        # rot = Fidtransform.transform.rotation
        # rgbcamlink_to_fid_trans = (trans.x, trans.y, trans.z)
        # rgbcamlink_to_fid_quat = (rot.x, rot.y, rot.z, rot.w)
        # rgbcamlink_to_fid_transform = t.concatenate_matrices(
        #     t.translation_matrix(rgbcamlink_to_fid_trans),
        #     t.translation_matrix(rgbcamlink_to_fid_quat)
        # )
        # fid_to_rgbcamlink_transform = t.inverse_matrix(rgbcamlink_to_fid_transform)

        # br.sendTransform(
        #     t.translation_from_matrix(fid_to_rgbcamlink_transform),
        #     t.quaternion_from_matrix(fid_to_rgbcamlink_transform),
        #     rospy.Time.now(),
        #     '{}_fid_{}'.format(camera_name, Fidtransform.fiducial_id),
        #     '{}_rgb_camera_link'.format(camera_name))
        
    
    if marker_idx_0 == None or marker_idx_1 == None:
        rospy.logwarn("there is missing markers - \
            id 0: {}, id 1: {}".format(marker_idx_0, marker_idx_1))
    else:
        marker_trans_0 = msg.transforms[marker_idx_0].transform.translation
        marker_rot_0 = msg.transforms[marker_idx_0].transform.rotation
        marker_trans_1 = msg.transforms[marker_idx_1].transform.translation
        marker_rot_1 = msg.transforms[marker_idx_1].transform.rotation
        # map is average between marker 0 and marker 1
        # compute rgb_camera_link --> map
        rgbcamlink_to_map_trans = (marker_trans_0.x/2 + marker_trans_1.x/2, 
                marker_trans_0.y/2 + marker_trans_1.y/2,
                marker_trans_0.z/2 + marker_trans_1.z/2)
        rgbcamlink_to_map_quat = averageQuaternions(
            np.array([[marker_rot_0.x, marker_rot_0.y, 
                        marker_rot_0.z, marker_rot_0.w],
                    [marker_rot_1.x, marker_rot_1.y, 
                    marker_rot_1.z, marker_rot_1.w]]))
        rgbcamlink_to_map_transform = t.concatenate_matrices(
                t.translation_matrix(rgbcamlink_to_map_trans), 
                t.quaternion_matrix(rgbcamlink_to_map_quat))

        # get camera_base -> rgb_camera_link
        try:
            (cambase_to_rgbcamlink_trans, cambase_to_rgbcamlink_rot) = \
                listener.lookupTransform('{}_rgb_camera_link'.format(camera_name), \
                    '{}_camera_base'.format(camera_name), rospy.Time.now() )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(e)            
            return
        cambase_to_rgbcamlink_transform = t.concatenate_matrices(
                t.translation_matrix(cambase_to_rgbcamlink_trans), 
                t.quaternion_matrix(cambase_to_rgbcamlink_rot))
        # compute and broadcast camera_base -> rgb_camera_link -> map
        cambase_to_map_transform = np.matmul(cambase_to_rgbcamlink_transform, rgbcamlink_to_map_transform)
        map_to_cambase_transform = t.inverse_matrix(cambase_to_map_transform)


        br.sendTransform(
            t.translation_from_matrix(map_to_cambase_transform),
            t.quaternion_from_matrix(map_to_cambase_transform),
            rospy.Time.now(),
            '{}_camera_base'.format(camera_name),
            'map')

        # map_to_rgbcamlink_transform = t.inverse_matrix(rgbcamlink_to_map_transform)
        # rgbcamlink_to_fid_transform = t.concatenate_matrices(
        #     t.translation_matrix((msg.transforms[marker_idx_0].transform.translation.x,
        #     msg.transforms[marker_idx_0].transform.translation.y,
        #     msg.transforms[marker_idx_0].transform.translation.z)),
        #     t.quaternion_matrix((
        #         msg.transforms[marker_idx_0].transform.rotation.x,
        #         msg.transforms[marker_idx_0].transform.rotation.y,
        #         msg.transforms[marker_idx_0].transform.rotation.z,
        #         msg.transforms[marker_idx_0].transform.rotation.w
        #     )),
        # )
        # map_to_fid_transform = np.matmul(map_to_rgbcamlink_transform, rgbcamlink_to_fid_transform)

        # br.sendTransform(
        #     t.translation_from_matrix(map_to_fid_transform),
        #     t.quaternion_from_matrix(map_to_fid_transform),
        #     rospy.Time.now(),
        #     '{}_fid_{}'.format(camera_name, Fidtransform.fiducial_id),
        #     'map')


if __name__ == '__main__':

    rospy.init_node('camera_tf_broadcaster')
    camera_name = rospy.get_param('~camera')
    rospy.Subscriber('/{}/fiducial_transforms'.format(camera_name),
                    FiducialTransformArray,
                    handle_camera_pose,
                    camera_name)
    rospy.spin()