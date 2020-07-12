#!/usr/bin/env python

import rospy
import rosnode
from zivid_camera.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Image
import numpy as np
from fiducial_msgs.msg import FiducialTransformArray
from assembly_camera_manager.srv import ExtrinsicCalibrateZivid
import tf
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
        self.extrinsic_calibration_service = rospy.Service('/zivid_camera/extrinsic_calibration', ExtrinsicCalibrateZivid, self.extrinsic_calibrate)
        

    def capture_assistant_suggest_settings(self):
        max_capture_time = rospy.Duration.from_sec(10) # 0.2 to 10s
        rospy.loginfo(
            "Calling capture assistant service with max capture time = %.2f sec",
            max_capture_time.to_sec(),
        )
        self.capture_assistant_service(
            max_capture_time=max_capture_time,
            ambient_light_frequency=CaptureAssistantSuggestSettingsRequest.AMBIENT_LIGHT_FREQUENCY_NONE,
        )

    def capture(self):
        rospy.loginfo("Calling capture service")
        self.capture_service()
    
    def extrinsic_calibrate(self, msg):

        rospy.loginfo("Calling extrinsic_calibrate service")
        target_fid_id = msg.fiducial_id
        self.capture()
        self.aruco_sub = rospy.Subscriber('/zivid_camera/fiducial_transforms', \
                            FiducialTransformArray, self.fiducial_to_map, target_fid_id)


    def fiducial_to_map(self, msg, target_fiducial_id):

        br = tf2_ros.StaticTransformBroadcaster()

        for i, Fidtransform in enumerate(msg.transforms): 
            # publish target fiducials as tf
            if Fidtransform.fiducial_id == target_fiducial_id:

                trans = Fidtransform.transform.translation
                rot = Fidtransform.transform.rotation

                static_tf = geometry_msgs.msg.TransformStamped()
                static_tf.header.stamp = rospy.Time.now()
                static_tf.header.frame_id = str(msg.header.frame_id,)
                static_tf.child_frame_id = "zivid_camera_fid_{}".format(Fidtransform.fiducial_id)

                static_tf.transform.translation.x = trans.x
                static_tf.transform.translation.y = trans.y
                static_tf.transform.translation.z = trans.z

                static_tf.transform.rotation.x = rot.x
                static_tf.transform.rotation.y = rot.y
                static_tf.transform.rotation.z = rot.z
                static_tf.transform.rotation.w = rot.w
                br.sendTransform(static_tf)

                rospy.loginfo_once("published static tf: {} -> zivid_camera_fid_{}".format(\
                    msg.header.frame_id, Fidtransform.fiducial_id))



if __name__ == "__main__":
    
    zivid_manager = ZividManager()
    zivid_manager.capture_assistant_suggest_settings()
    rospy.spin()

