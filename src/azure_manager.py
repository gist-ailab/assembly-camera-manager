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
        self.filter_size = 10

    def calibrate_azure(self, msg):

        self.is_sucess = True
        self.is_finish = False
        rospy.loginfo("Calibrating {}".format(self.camera_name))
        target_fid_id = msg.target_fiducial_ids

        # get transform (fiducial -> map) N times
        self.cam_to_fids = []
        self.trans_dict = {}
        self.quat_dict = {}
        self.img_err_dict = {}
        self.obj_err_dict = {}
        self.n_sucess_dict = {}
        self.n_iter = 0
        self.aruco_sub = rospy.Subscriber('/{}/fiducial_transforms'.format(self.camera_name), \
                                FiducialTransformArray, self.get_fiducial_to_map, target_fid_id)
        while self.n_iter != self.filter_size:
            rospy.sleep(0.1)

        for fiducial_id in self.n_sucess_dict.keys():
            rospy.loginfo("fid id: {} ==> n_sucess: {}/{}".format(fiducial_id, self.n_sucess_dict[fiducial_id], self.filter_size))
            errors = self.img_err_dict[fiducial_id]
            rospy.loginfo("img err =>  median: {:.3f} \t std: {:.3f}".format(np.median(errors), np.std(errors)))
            errors = self.obj_err_dict[fiducial_id]
            rospy.loginfo("obj err =>  median: {:.3f} \t std: {:.3f}".format(np.median(errors), np.std(errors)))

            if self.n_sucess_dict[fiducial_id] != self.filter_size:
                self.is_sucess = False
        
        # apply median filter
        for fiducial_id in  self.trans_dict.keys():
            trans_median = np.median(np.asarray(self.trans_dict[fiducial_id]), axis=0)
            quat_median = np.median(np.asarray(self.quat_dict[fiducial_id]), axis=0)

            static_tf = geometry_msgs.msg.TransformStamped()
            static_tf.header.frame_id = "{}".format(self.header_frame_id)
            static_tf.child_frame_id = "{}_camera_fid_{}".format(self.camera_name, fiducial_id)
            
            static_tf.transform.translation.x = trans_median[0]
            static_tf.transform.translation.y = trans_median[1]
            static_tf.transform.translation.z = trans_median[2]

            static_tf.transform.rotation.x = quat_median[0]
            static_tf.transform.rotation.y = quat_median[1]
            static_tf.transform.rotation.z = quat_median[2]
            static_tf.transform.rotation.w = quat_median[3]

            rospy.loginfo("published static tf: {} -> {}_camera_fid_{}".format(\
               self.header_frame_id, self.camera_name, fiducial_id, ))   
            self.cam_to_fids.append(static_tf)

        return self.is_sucess 


    def get_fiducial_to_map(self, msg, target_fiducial_ids):
        self.n_target_fiducial_ids = len(target_fiducial_ids)
        self.header_frame_id = msg.header.frame_id

        for i, Fidtransform in enumerate(msg.transforms): 
            # publish target fiducials as tf
            target_fiducial_id = Fidtransform.fiducial_id

            if target_fiducial_id in target_fiducial_ids:
                trans = Fidtransform.transform.translation
                rot = Fidtransform.transform.rotation
                # initialize dict
                if target_fiducial_id not in self.trans_dict:
                    self.trans_dict[target_fiducial_id] = [[trans.x, trans.y, trans.z]]
                    self.quat_dict[target_fiducial_id] = [[rot.x, rot.y, rot.z, rot.w]]
                    self.n_sucess_dict[target_fiducial_id] = 1
                    self.img_err_dict[target_fiducial_id] = [Fidtransform.image_error]
                    self.obj_err_dict[target_fiducial_id] = [Fidtransform.object_error]
                else:
                    self.trans_dict[target_fiducial_id].append([trans.x, trans.y, trans.z])
                    self.quat_dict[target_fiducial_id].append([rot.x, rot.y, rot.z, rot.w])
                    self.n_sucess_dict[target_fiducial_id] += 1
                    self.img_err_dict[target_fiducial_id].append(Fidtransform.image_error)
                    self.obj_err_dict[target_fiducial_id].append(Fidtransform.object_error)

        self.n_iter += 1
        if self.n_iter == self.filter_size:
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
