#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Image
import numpy as np
from fiducial_msgs.msg import FiducialArray
import tf
import message_filters
from tf import transformations as t
import tf2_ros
import geometry_msgs
import sys
sys.path.append('/home/demo/Workspace/seung/open3d_ros_helper')
from open3d_ros_helper.utils import *
import time
import ros_numpy
import open3d
import cv2
from assembly_camera_manager.srv import ExtrinsicCalibrate, SetCamPose
from rospy_message_converter import json_message_converter
import json
import os
import datetime
from tf2_geometry_msgs import do_transform_pose


class PointCloudManager:

    def __init__(self):

        rospy.init_node("point_cloud_manager", anonymous=True)
        rospy.loginfo("Starting point_cloud_manager.py")

        triple_calib_srv = rospy.Service('/triple_azure/extrinsic_calibration', ExtrinsicCalibrate, self.calibrate_triple_azure)
        
        self.marker_pub = rospy.Publisher('/points2/marker', PointCloud2, queue_size=1)
        self.marker_icp_pub = rospy.Publisher('/points2/marker_icp', PointCloud2, queue_size=1)
        self.merged_cloud_pub = rospy.Publisher('/points2/merged', PointCloud2, queue_size=1)


        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        # min, max bounds for x, y, z in meter
        self.ROI = {'x': [rospy.get_param('~x_min'), rospy.get_param('~x_max')], 
                    'y': [rospy.get_param('~y_min'), rospy.get_param('~y_max')],
                    'z': [rospy.get_param('~z_min'), rospy.get_param('~z_max')]} 
                    
        self.img_width = rospy.get_param('~img_width') 
        self.img_height = rospy.get_param('~img_height') 
        self.vox_size = rospy.get_param('~vox_size') 
        self.fiducial_ids = [int(x) for x in rospy.get_param('~target_fiducial_ids').split(',')]
        self.save_folder = rospy.get_param('~save_folder') 


        rospy.loginfo("target_fiducials : {}".format(self.fiducial_ids))
        self.target_fiducial_id = self.fiducial_ids[0]
        self.source_fiducial_ids = self.fiducial_ids[1:]
        self.icp_results = dict.fromkeys(self.source_fiducial_ids)
        self.br = tf2_ros.StaticTransformBroadcaster()

    def calibrate_triple_azure(self, msg):

        self.is_finish = False
        rospy.loginfo("Initial calibration between azures")
        calibrate_1 = rospy.ServiceProxy('/azure1/extrinsic_calibration', ExtrinsicCalibrate)
        calibrate_2 = rospy.ServiceProxy('/azure2/extrinsic_calibration', ExtrinsicCalibrate)
        calibrate_3 = rospy.ServiceProxy('/azure3/extrinsic_calibration', ExtrinsicCalibrate)
        
        # initial calibration
        all_sucess = False
        while not all_sucess:
            is_sucess_1 = calibrate_1(self.fiducial_ids)
            is_sucess_2 = calibrate_2(self.fiducial_ids) 
            is_sucess_3 = calibrate_3(self.fiducial_ids) 
            all_sucess = is_sucess_1 or is_sucess_2 or is_sucess_3
        rospy.sleep(1.0)

        # refine the calibration using ICP b/w markers
        rospy.loginfo("ICP b/w markers for calibration refinement ")
        points_sub_1 = message_filters.Subscriber('/azure1/points2', PointCloud2, buff_size=1280*720*3)
        points_sub_2 = message_filters.Subscriber('/azure2/points2', PointCloud2, buff_size=1280*720*3)
        points_sub_3 = message_filters.Subscriber('/azure3/points2', PointCloud2, buff_size=1280*720*3)
        fidvertices_sub_1 = message_filters.Subscriber('/azure1/fiducial_vertices', FiducialArray)
        fidvertices_sub_2 = message_filters.Subscriber('/azure2/fiducial_vertices', FiducialArray)
        fidvertices_sub_3 = message_filters.Subscriber('/azure3/fiducial_vertices', FiducialArray)
        self.icp_subs = [points_sub_1, points_sub_2, points_sub_3, 
                            fidvertices_sub_1, fidvertices_sub_2, fidvertices_sub_3]
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [points_sub_1, points_sub_2, points_sub_3, fidvertices_sub_1, fidvertices_sub_2, fidvertices_sub_3], queue_size=1, slop=2)
        self.ts.registerCallback(self.icp_with_markers)

        # wait for ICP results
        while not self.is_finish:
            pass
        rospy.loginfo("Extrinisic calibration finished ")

        if rospy.get_param('~publish_cloud'):
            # merge pointclouds and publish it
            rospy.loginfo("Merging pointclouds according to the ICP results")
            points_sub_1 = message_filters.Subscriber('/azure1/points2', PointCloud2, buff_size=1280*720*3)
            points_sub_2 = message_filters.Subscriber('/azure2/points2', PointCloud2, buff_size=1280*720*3)
            points_sub_3 = message_filters.Subscriber('/azure3/points2', PointCloud2, buff_size=1280*720*3)
            self.merge_subscribers = [points_sub_1, points_sub_2, points_sub_3]
            self.ts = message_filters.ApproximateTimeSynchronizer(
                [points_sub_1, points_sub_2, points_sub_3], queue_size=1, slop=1)
            self.ts.registerCallback(self.merge_pointcloud)
        
        return self.is_finish


    def icp_with_markers(self, pcl_msg_1, pcl_msg_2, pcl_msg_3, fidvertices_1, fidvertices_2, fidvertices_3):
        try: 
            transform_1_to_map = self.tf_buffer.lookup_transform("map", "azure1_rgb_camera_link", rospy.Time(), rospy.Duration(1.0))
            transform_2_to_1 = self.tf_buffer.lookup_transform("azure1_rgb_camera_link", "azure2_rgb_camera_link", rospy.Time(), rospy.Duration(1.0))
            transform_3_to_1 = self.tf_buffer.lookup_transform("azure1_rgb_camera_link", "azure3_rgb_camera_link", rospy.Time(), rospy.Duration(1.0)) 

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            return
        start_time = time.time()
        mask_1 = self.fiducial_vertices_to_mask(fidvertices_1)
        mask_2 = self.fiducial_vertices_to_mask(fidvertices_2)
        mask_3 = self.fiducial_vertices_to_mask(fidvertices_3)
        # crop the point clouds corresponding to the markers and transform it to cloud 1
        cloud_1 = convert_ros_to_o3d(pcl_msg_1)
        cloud_1 = crop_o3d_cloud_with_mask(cloud_1, mask_1)
        cloud_2 = convert_ros_to_o3d(pcl_msg_2)
        cloud_2 = crop_o3d_cloud_with_mask(cloud_2, mask_2)
        cloud_2 = do_transform_o3d_cloud(cloud_2, transform_2_to_1)
        cloud_3 = convert_ros_to_o3d(pcl_msg_3)
        cloud_3 = crop_o3d_cloud_with_mask(cloud_3, mask_3)
        cloud_3 = do_transform_o3d_cloud(cloud_3, transform_3_to_1)
        # merged the point clouds and transform it to the map for visualization
        cloud_markers = [cloud_1, cloud_2, cloud_3]
        cloud_marker = cloud_1 + cloud_2 + cloud_3
        cloud_marker = do_transform_o3d_cloud(cloud_marker, transform_1_to_map)
        pcl_msg_marker = convert_o3d_to_ros(cloud_marker, frame_id="map")
        self.marker_pub.publish(pcl_msg_marker)

        target_cloud = cloud_1
        source_clouds = [cloud_2, cloud_3]
        refined_cloud_marker = cloud_1
        # icp refinement from cloud 2, 3 to cloud 1
        while len(source_clouds) != 0:
            scores = []
            for i, source_cloud in enumerate(source_clouds):
                icp_result, evaluation = icp_refinement(source_cloud, target_cloud, max_correspondence_distance=5)
                rospy.loginfo("icp_result for marker id {}: {}".format(self.source_fiducial_ids[i], evaluation))
                scores.append(icp_result.inlier_rmse)
            best_source_cloud = source_clouds[np.argmin(scores)]
            best_source_cloud.transform(icp_result.transformation)
            refined_cloud_marker += best_source_cloud
            del source_clouds[np.argmin(scores)]
            self.icp_results[self.source_fiducial_ids[i]] = icp_result
            
        refined_cloud_marker = do_transform_o3d_cloud(refined_cloud_marker, transform_1_to_map)
        pcl_msg_refined_marker = convert_o3d_to_ros(refined_cloud_marker, frame_id="map")
        self.marker_icp_pub.publish(pcl_msg_refined_marker)
        self.is_finish = True

        # publish refined transform from each camera to target fid
        transform_2_to_1 = self.tf_buffer.lookup_transform("azure1_rgb_camera_link", "azure2_rgb_camera_link", rospy.Time(), rospy.Duration(1.0))
        
        mat_2_to_1 = msg_to_se3(transform_2_to_1)
        mat_icp = self.icp_results[self.source_fiducial_ids[0]].transformation
        mat_2_to_1_icp = np.dot(mat_2_to_1, mat_icp)
        transform_stamped_2_to_1_icp = se3_to_transform_stamped(mat_2_to_1_icp, "azure2_rgb_camera_link", "azure1_rgb_camera_link")
        self.br.sendTransform(transform_stamped_2_to_1_icp)   
        rospy.loginfo("published refined static tf: {} -> {}".format(transform_2_to_1.header.frame_id, transform_2_to_1.child_frame_id))   

        mat_3_to_1 = msg_to_se3(transform_3_to_1)
        mat_icp = self.icp_results[self.source_fiducial_ids[1]].transformation
        mat_3_to_1_icp = np.dot(mat_3_to_1, mat_icp)
        transform_stamped_3_to_1_icp = se3_to_transform_stamped(mat_2_to_1_icp, "azure3_rgb_camera_link", "azure1_rgb_camera_link")
        self.br.sendTransform(transform_stamped_3_to_1_icp)   
        rospy.loginfo("published refined static tf: {} -> {}".format(transform_3_to_1.header.frame_id, transform_3_to_1.child_frame_id))   

        # save calibration results
        header = str(datetime.datetime.now()).split('.')[0]
        self.save_transfrom_as_json(header, "map", "azure1_rgb_camera_link")
        self.save_transfrom_as_json(header, "map", "azure2_rgb_camera_link")
        self.save_transfrom_as_json(header, "map", "azure3_rgb_camera_link")

        for sub in self.icp_subs: sub.unregister()

    def save_transfrom_as_json(self, header, source_frame, target_frame):
        # save transform from source to target as json file at the save_folder
        transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(), rospy.Duration(1.0))
        with open(os.path.join(self.save_folder, "{}_{}_to_{}.json".format(header, source_frame, target_frame)), "w") as json_file:
            json.dump(json_message_converter.convert_ros_message_to_json(transform), json_file)


    def merge_pointcloud(self, pcl_msg_1, pcl_msg_2, pcl_msg_3):
        # transform cloud 2 & 3 to the frame of cloud1     
        try: 
            transform_1_to_map = self.tf_buffer.lookup_transform("map", "azure1_rgb_camera_link", rospy.Time(), rospy.Duration(1.0))
            transform_2_to_1 = self.tf_buffer.lookup_transform("azure1_rgb_camera_link", "azure2_rgb_camera_link", rospy.Time(), rospy.Duration(1.0))
            transform_3_to_1 = self.tf_buffer.lookup_transform("azure1_rgb_camera_link", "azure3_rgb_camera_link", rospy.Time(), rospy.Duration(1.0)) 

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        start_time = time.time()

        cloud_1 = convert_ros_to_o3d(pcl_msg_1, remove_nans=True)
        cloud_1 = cloud_1.voxel_down_sample(voxel_size=self.vox_size)

        cloud_2 = convert_ros_to_o3d(pcl_msg_2, remove_nans=True)
        cloud_2 = cloud_2.voxel_down_sample(voxel_size=self.vox_size)
        cloud_2 = do_transform_o3d_cloud(cloud_2, transform_2_to_1)
        cloud_2.transform(self.icp_results[self.source_fiducial_ids[0]].transformation)

        cloud_3 = convert_ros_to_o3d(pcl_msg_3, remove_nans=True)
        cloud_3 = cloud_3.voxel_down_sample(voxel_size=self.vox_size)
        cloud_3 = do_transform_o3d_cloud(cloud_3, transform_3_to_1)
        cloud_3.transform(self.icp_results[self.source_fiducial_ids[1]].transformation)

        cloud_merged = cloud_1 + cloud_2 + cloud_3
        cloud_merged = do_transform_o3d_cloud(cloud_merged, transform_1_to_map)
        cloud_merged = o3d_cloud_pass_through_filter(cloud_merged, ROI=self.ROI)
        # plane_model, inliers = cloud_merged.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=30)
        # np_cloud = np.asarray(cloud_merged.points)
        # outliers = np.ones(len(np_cloud), np.bool)
        # outliers[inliers] = 0
        # cloud_merged.points = open3d.utility.Vector3dVector(np_cloud[outliers])
        pcl_msg_merged = convert_o3d_to_ros(cloud_merged, frame_id="map")
        rospy.loginfo("mering pointclouds: {}".format(time.time()-start_time))
        self.merged_cloud_pub.publish(pcl_msg_merged)

    def fiducial_vertices_to_mask(self, fiducial_vertices):

        is_sucess = False
        while not is_sucess:
            n_sucess = 0
            mask = np.zeros((self.img_height, self.img_width))
            for fiducial in fiducial_vertices.fiducials:
                if fiducial.fiducial_id in self.fiducial_ids:
                    target_fid = fiducial
                    pts = np.array([[target_fid.x0, target_fid.y0], 
                                    [target_fid.x1, target_fid.y1], 
                                    [target_fid.x2, target_fid.y2], 
                                    [target_fid.x3, target_fid.y3]], np.int32)
                    mask = cv2.fillPoly(mask, [pts], 255)
                    rospy.sleep(0.5)
                    n_sucess += 1
            if n_sucess == len(self.fiducial_ids): is_sucess = True        
        return mask


if __name__ == "__main__":
    
    point_cloud_manager = PointCloudManager()
    rospy.spin()
