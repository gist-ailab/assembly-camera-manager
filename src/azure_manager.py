#!/usr/bin/env python

import rospy
import roslib
from fiducial_msgs.msg import FiducialTransformArray
from std_msgs.msg import String
from assembly_camera_manager.srv import ExtrinsicCalibrate
import tf.transformations as tf_trans

import tf
import tf2_ros

import numpy as np
import geometry_msgs.msg

class AzureManager:

    def __init__(self):

        rospy.init_node("azure_manager", anonymous=True)
        self.camera_name = rospy.get_param('~camera_name')
        calib_srv = rospy.Service('/{}/extrinsic_calibration'.format(self.camera_name), ExtrinsicCalibrate, self.calibrate_azure)
        rospy.loginfo("Starting azure_manager.py for {}".format(self.camera_name))
        self.cam_to_fids = []

    def calibrate_azure(self, msg):

        self.is_sucess = False
        self.is_finish = False
        rospy.loginfo("Calibrating {}".format(self.camera_name))
        target_fid_id = msg.target_fiducial_ids
        self.aruco_sub = rospy.Subscriber('/{}/fiducial_transforms'.format(self.camera_name), \
                            FiducialTransformArray, self.fiducial_to_map, target_fid_id)
        while not self.is_finish:
            pass
        return self.is_sucess 


    def fiducial_to_map(self, msg, target_fiducial_ids):
        self.cam_to_fids = []
        n_sucess = 0
        for i, Fidtransform in enumerate(msg.transforms): 
            # publish target fiducials as tf
            if Fidtransform.fiducial_id in target_fiducial_ids:

                static_tf = geometry_msgs.msg.TransformStamped()
                static_tf.header.frame_id = "{}".format(msg.header.frame_id)
                static_tf.child_frame_id = "{}_camera_fid_{}".format(self.camera_name, Fidtransform.fiducial_id)

                trans = Fidtransform.transform.translation
                static_tf.transform.translation.x = trans.x
                static_tf.transform.translation.y = trans.y
                static_tf.transform.translation.z = trans.z

                rot = Fidtransform.transform.rotation
                static_tf.transform.rotation.x = rot.x
                static_tf.transform.rotation.y = rot.y
                static_tf.transform.rotation.z = rot.z
                static_tf.transform.rotation.w = rot.w
                
                # get cam_to_map
                H_cam_to_map = np.eye(4)
                H_cam_to_map[:3, 3] = tf_trans.translation_matrix((trans.x, trans.y, trans.z))[:3, 3]
                H_cam_to_map[:3, :3] = tf_trans.quaternion_matrix((rot.x, rot.y, rot.z, rot.w))[:3, :3]

                rospy.loginfo("published static tf: {} -> {}_camera_fid_{}".format(\
                    msg.header.frame_id, self.camera_name, Fidtransform.fiducial_id, ))   
                self.cam_to_fids.append(static_tf)
                n_sucess += 1

        if n_sucess == len(target_fiducial_ids): self.is_sucess = True
        self.is_finish = True
        self.aruco_sub.unregister()

    def publish_map(self):
        if len(self.cam_to_fids) != 0:
            for cam_to_fid in self.cam_to_fids:
                br = tf2_ros.StaticTransformBroadcaster()
                cam_to_fid.header.stamp = rospy.Time.now()
                br.sendTransform(cam_to_fid)
            rospy.sleep(0.05)

if __name__ == '__main__':

    azure_manager = AzureManager()
    while not rospy.is_shutdown():
        azure_manager.publish_map()
