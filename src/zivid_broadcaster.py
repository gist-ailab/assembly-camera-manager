#!/usr/bin/env python

import rospy
import rosnode
from zivid_camera.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Image
import cv2, cv_bridge
import numpy as np
from fiducial_msgs.msg import FiducialTransformArray
import tf


class Receiver:

    def __init__(self):

        rospy.init_node("zivid_receiver", anonymous=True)
        rospy.loginfo("Starting zivid_broadcaster.py")
        camera_name = rospy.get_param('~camera')

        ca_suggest_settings_service = "/zivid_camera/capture_assistant/suggest_settings"
        rospy.wait_for_service(ca_suggest_settings_service, 30.0)

        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)

        self.capture_assistant_service = rospy.ServiceProxy(
            ca_suggest_settings_service, CaptureAssistantSuggestSettings
        )
        self.capture_service = rospy.ServiceProxy("/zivid_camera/capture", Capture)
        self.aruco_sub = rospy.Subscriber('/{}/fiducial_transforms'.format(camera_name),
                    FiducialTransformArray,
                    self.aruco_to_map,
                    camera_name)
        self.target_fiducial_ids = [24, 25, 26]


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

    def aruco_to_map(self, msg, camera_name):
        br = tf.TransformBroadcaster()
        for i, Fidtransform in enumerate(msg.transforms): 
            # publish all target fiducials as tf
            if Fidtransform.fiducial_id in self.target_fiducial_ids:
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


if __name__ == "__main__":
    
    receiver = Receiver()
    receiver.capture_assistant_suggest_settings()

    # rospy.spin()
    while True:
        receiver.capture()