#!/usr/bin/env python

import rospy
import rosnode
from zivid_camera.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Image
import numpy as np
from fiducial_msgs.msg import FiducialTransformArray
from assembly_camera_manager.srv import ExtrinsicCalibrate
import tf
from tf import transformations as t
import tf2_ros
import geometry_msgs


class ZividManager:

    def __init__(self):

        rospy.init_node("zivid_manager", anonymous=True)
        rospy.loginfo("Starting zivid_manager.py")
        # get suggested settings for Zivid
        ca_suggest_settings_service = "/zivid_camera/capture_assistant/suggest_settings"
        rospy.wait_for_service(ca_suggest_settings_service, 30.0)
        self.capture_assistant_service = rospy.ServiceProxy(
            ca_suggest_settings_service, CaptureAssistantSuggestSettings
        )
        # ready to capture and launch a extrinsic calibration server
        self.capture_service = rospy.ServiceProxy("/zivid_camera/capture", Capture)        
        self.extrinsic_calibration_service = rospy.Service('/zivid_camera/extrinsic_calibration', ExtrinsicCalibrate, self.extrinsic_calibrate)

    def capture_assistant_suggest_settings(self):
        max_capture_time = rospy.Duration.from_sec(1.0) # 0.2 to 10s
        rospy.loginfo(
            "Calling capture assistant service with max capture time = %.2f sec",
            max_capture_time.to_sec(),
        )
        self.capture_assistant_service(
            max_capture_time=max_capture_time,
            ambient_light_frequency=CaptureAssistantSuggestSettingsRequest.AMBIENT_LIGHT_FREQUENCY_NONE,
        )

    def capture(self):
        rospy.loginfo_once("Calling capture service")
        self.capture_service()
    
    def extrinsic_calibrate(self, msg):

        self.is_sucess = False
        self.is_finish = False
        rospy.loginfo("Calling extrinsic_calibrate service")
        target_fiducial_id = msg.target_fiducial_id
        self.zivid_sub = rospy.Subscriber('/zivid_camera/fiducial_transforms', \
                            FiducialTransformArray, self.fiducial_to_map, target_fiducial_id)
        while not self.is_finish:
            pass
        return self.is_sucess 

    def fiducial_to_map(self, msg, target_fiducial_id):

        br = tf2_ros.StaticTransformBroadcaster()

        for i, Fidtransform in enumerate(msg.transforms): 
            # publish target fiducials as tf
            if Fidtransform.fiducial_id == target_fiducial_id:

                static_tf = geometry_msgs.msg.TransformStamped()
                static_tf.header.stamp = rospy.Time.now()
                static_tf.child_frame_id = "{}".format(msg.header.frame_id)
                static_tf.header.frame_id = "zivid_camera_fid_{}".format(Fidtransform.fiducial_id)
                
                trans = Fidtransform.transform.translation 
                trans = (trans.x, trans.y, trans.z)               
                rot = Fidtransform.transform.rotation
                rot = (rot.x, rot.y, rot.z, rot.w)
                transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
                inversed_transform = t.inverse_matrix(transform)
                # inverse transform
                trans = t.translation_from_matrix(inversed_transform)
                rot = t.quaternion_from_matrix(inversed_transform)

                static_tf.transform.translation.x = trans[0]
                static_tf.transform.translation.y = trans[1]
                static_tf.transform.translation.z = trans[2]
                static_tf.transform.rotation.x = rot[0]
                static_tf.transform.rotation.y = rot[1]
                static_tf.transform.rotation.z = rot[2]
                static_tf.transform.rotation.w = rot[3]

                # static_tf = transform.inversematrix(static_tf)
                br.sendTransform(static_tf)
                rospy.loginfo_once("published static tf: {} -> zivid_camera_fid_{}".format(\
                    msg.header.frame_id, Fidtransform.fiducial_id))
                self.is_sucess = True

        self.zivid_sub.unregister()
        self.is_finish = True

if __name__ == "__main__":
    
    zivid_manager = ZividManager()
    zivid_manager.capture_assistant_suggest_settings()
    while True:
        zivid_manager.capture()

