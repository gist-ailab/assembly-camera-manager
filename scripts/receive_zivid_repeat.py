import rospy
import rosnode
from zivid_camera.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Image
import cv2, cv_bridge
import numpy as np


class Receiver:

    def __init__(self):

        rospy.init_node("zivid_receiver", anonymous=True)
        rospy.loginfo("Starting zivid_receiver.py")

        ca_suggest_settings_service = "/zivid_camera/capture_assistant/suggest_settings"
        rospy.wait_for_service(ca_suggest_settings_service, 30.0)

        self.bridge = cv_bridge.CvBridge()

        self.capture_assistant_service = rospy.ServiceProxy(
            ca_suggest_settings_service, CaptureAssistantSuggestSettings
        )
        self.capture_service = rospy.ServiceProxy("/zivid_camera/capture", Capture)

       
        # self.rgb_sub = rospy.Subscriber('rgb/image_raw', Image, self.rgb_callback)
        # self.depth_sub = rospy.Subscriber('depth_to_rgb/image_raw', Image, self.depth_callback)

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


if __name__ == "__main__":
    
    receiver = Receiver()
    receiver.capture_assistant_suggest_settings()

    while True:
        receiver.capture()