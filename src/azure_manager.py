#!/usr/bin/env python

import rospy
import roslib
from fiducial_msgs.msg import FiducialTransformArray
from std_msgs.msg import String
from assembly_camera_manager.srv import GetCameraPoseSingleMarker, GetCameraPoseMultipleMarker, SetCameraPose
import tf.transformations as tf_trans

import tf
import tf2_ros

import numpy as np
import json
import geometry_msgs.msg
from rospy_message_converter import json_message_converter
import os
import sys
import yaml
from open3d_ros_helper import open3d_ros_helper as orh
import PyKDL
import time

class AzureManager:

    def __init__(self):

        rospy.init_node("azure_manager", anonymous=True)
        self.camera_name = rospy.get_param('~camera_name')
        self.camera_map = rospy.get_param('~camera_map')
        with open(rospy.get_param('~world_map')) as f:
            self.world_map = yaml.load(f, Loader=yaml.FullLoader)

        getcamerapose_singlemarker_srv = rospy.Service('/{}/get_camera_pose_single_marker'
                    .format(self.camera_name), GetCameraPoseSingleMarker, self.get_camera_pose_from_single_marker)
        getcamerapose_multiplemarker_srv = rospy.Service('/{}/get_camera_pose_multiple_marker'
                    .format(self.camera_name), GetCameraPoseMultipleMarker, self.get_camera_pose_from_multiple_marker)
        setcamerapose_srv = rospy.Service('/{}/set_camera_pose'
                    .format(self.camera_name), SetCameraPose, self.set_camera_pose)

        self.static_aruco_tfs = []
        self.static_world_tfs = []
        self.br = tf2_ros.StaticTransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo("Starting azure_manager.py for {}".format(self.camera_name))

    def get_camera_pose_from_single_marker(self, msg):

        target_id = msg.target_id
        n_frame = msg.n_frame
        img_err_thresh = msg.img_err_thresh
        obj_err_thresh = msg.obj_err_thresh
        rospy.loginfo("Get camera pose of {} for marker ID {}".format(self.camera_name, target_id))
        pos_list = []
        quat_list = []
        img_err_list = []
        obj_err_list = []
        n_sucess = 0

        # get transforms for n_frame
        for n in range(n_frame):
            fid_tfs = rospy.wait_for_message('/{}/fiducial_transforms'.format(self.camera_name), FiducialTransformArray)
            header_frame_id = fid_tfs.header.frame_id
            for i, fid_tf in enumerate(fid_tfs.transforms): 
                if fid_tf.fiducial_id == target_id:                
                    pos, quat = orh.transform_to_pq(fid_tf.transform)
                    pos_list.append(pos)
                    quat_list.append(quat)
                    img_err_list.append(fid_tf.image_error)
                    obj_err_list.append(fid_tf.object_error)
                    n_sucess += 1
        
        if len(pos_list) == 0:
            rospy.logwarn("Failed to detect the marker ID {}".format(target_id))
            return False
        
        # select the frame with minimum image error
        idx = np.argmin(img_err_list)
        rospy.loginfo("\t Marker ID {}: n_sucess={}/{}".format(target_id, n_sucess, n_frame))        
        if img_err_list[idx] > img_err_thresh: 
            rospy.logwarn("Reject marker ID {} (img err: {:.4f} > {:.4f})".format(target_id, img_err_list[idx], img_err_thresh))
            return False
        if obj_err_list[idx] > img_err_thresh: 
            rospy.logwarn("Reject marker ID {} (obj err: {:.4f} > {:.4f})".format(target_id, obj_err_list[idx], obj_err_thresh))
            return False
        else:
            rospy.loginfo("\t img err: {:.4f} \t obj err:{:.4f}".format(img_err_list[idx], obj_err_list[idx]))

        pos_min = pos_list[idx]
        quat_min = quat_list[idx]
        source_frame = "{}".format(header_frame_id)
        target_frame = "{}_camera_fid_{}".format(self.camera_name, target_id)
        static_tf_min = orh.pq_to_transform_stamped(pos_min, quat_min, source_frame, target_frame)
        self.static_aruco_tfs.append(static_tf_min)
        rospy.loginfo("Publish static tf: {} -> {}_camera_fid_{} from ArUco".format(header_frame_id, self.camera_name, target_id))   

        # find target marker in world map
        target_marker = None
        for marker in self.world_map["markers"]:
            if marker["id"] == target_id:
                target_marker = marker
        if target_marker is None: 
            rospy.logwarn("No information in world map for marker ID {}".format(target_id))
        
        pos = target_marker["position"]
        pos = [-p for p in pos]
        quat = target_marker["orientation"] # TODO: invert quaternion 
        source_frame = "{}_camera_fid_{}".format(self.camera_name, target_id)
        target_frame = "base"
        static_tf_base_to_fid = orh.pq_to_transform_stamped(pos, quat, source_frame, target_frame)
        self.static_world_tfs.append(static_tf_base_to_fid)

        if msg.publish_worldmap:
            rospy.loginfo("Publish static tf:{}_camera_fid_{} -> base from world map ".format(self.camera_name, target_id))   
            self.br.sendTransform(self.static_aruco_tfs + self.static_world_tfs)
        else:
            self.br.sendTransform(self.static_aruco_tfs)
        rospy.spin()
        return True 

        
    def get_camera_pose_from_multiple_marker(self, msg):
        
        self.static_aruco_tfs = []  # initialize static tf
        for target_id in msg.target_ids:
            getcamerapose_singlemarker = rospy.ServiceProxy('/{}/get_camera_pose_single_marker'.format(self.camera_name), GetCameraPoseSingleMarker)
            is_sucess = getcamerapose_singlemarker(False, target_id, msg.n_frame, msg.img_err_thresh, msg.obj_err_thresh)
        
        if msg.publish_worldmap:
            # get average of aruco map
            pos_list = []
            quat_list = []
            for aruco_tf in self.static_aruco_tfs:
                pos, quat = orh.transform_stamped_to_pq(aruco_tf)
                pos_list.append(pos)
                quat_list.append(quat)
            pos_aruco_avg, quat_aruco_avg = orh.average_pq(pos_list, quat_list)
            # calculate fid 0 to average of aruco map
            pos_fid, quat_fid = orh.transform_stamped_to_pq(self.static_aruco_tfs[0])
            pos_fid_to_avg = pos_fid - pos_aruco_avg
            quat_aruco_avg = PyKDL.Rotation.Quaternion(*quat_aruco_avg)
            quat_fid = PyKDL.Rotation.Quaternion(*quat_fid)
            quat_fid_to_avg = quat_aruco_avg * quat_fid.Inverse()

            # get corresponding tf from world map
            pos_list = []
            quat_list = []
            for world_tf in self.static_world_tfs:
                pos, quat = orh.transform_stamped_to_pq(world_tf)
                pos_list.append(pos)
                quat_list.append(quat)
            pos_base_to_avg, quat_base_to_avg = orh.average_pq(pos_list, quat_list)
            # calculate average of aruco map to world_base
            pos_avg_to_base = [p for p in pos_base_to_avg]
            quat_base_to_avg = PyKDL.Rotation.Quaternion(*quat_base_to_avg)
            quat_avg_to_base = quat_base_to_avg.Inverse()

            # aruco tf #1 to aruco tf average + aruco tf average to base
            pos_fid_to_base = [sum(p) for p in zip(pos_fid_to_avg, pos_avg_to_base)]
            quat_fid_to_base = quat_fid_to_avg * quat_avg_to_base
            quat_fid_to_base = quat_fid_to_base.GetQuaternion()
            source_frame = self.static_aruco_tfs[0].child_frame_id
            target_frame = "base"

            static_tf_fid_to_base = orh.pq_to_transform_stamped(pos_fid_to_base, quat_fid_to_base, source_frame, target_frame)
            self.static_world_tfs.append(static_tf_fid_to_base)
            self.br.sendTransform(self.static_aruco_tfs + self.static_world_tfs)
        self.save_transfrom_as_json("base", "{}_rgb_camera_link".format(self.camera_name))
        rospy.loginfo("Finished the camera pose calibration")
        return True

    def set_camera_pose(self, msg):

        with open(os.path.join(self.camera_map, msg.json_file + '.json'), "r") as json_file:
            json_str = json.load(json_file)

        self.static_aruco_tfs = []
        static_tf = json_message_converter.convert_json_to_ros_message('geometry_msgs/TransformStamped', json_str)
        static_tf.header.stamp = rospy.Time.now()       
        self.static_aruco_tfs.append(static_tf)
        self.br.sendTransform(self.static_aruco_tfs)

        rospy.loginfo("published static tf: {} -> {} from json".format(\
            static_tf.header.frame_id, static_tf.child_frame_id))   

        return True

    def save_transfrom_as_json(self, source_frame, target_frame):
        # save transform from source to target as json file at the save_folder
        transform = self.tf_buffer.lookup_transform(source_frame, target_frame, rospy.Time(), rospy.Duration(1.0))
        with open(os.path.join(self.camera_map, "{}_to_{}_{}.json".format(source_frame, target_frame, time.strftime("%Y%m%d-%H%M%S"))), "w") as json_file:
            json.dump(json_message_converter.convert_ros_message_to_json(transform), json_file)



if __name__ == '__main__':

    azure_manager = AzureManager()
    rospy.spin()
