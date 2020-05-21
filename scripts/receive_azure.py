import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np
import message_filters
import getch

class Receiver:

    def __init__(self):
        
        rospy.init_node('azure_receiver')
        rospy.loginfo("Starting azure_receiver.py")

        self.bridge = cv_bridge.CvBridge()

        rgb_sub = message_filters.Subscriber('rgb/image_raw', Image)
        depth_sub = message_filters.Subscriber('depth_to_rgb/image_raw', Image, buff_size=1536*2048*6)
        self.ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=1, slop=0.1)
        rospy.loginfo("[INFO] Starting rgb-d subscriber with time synchronizer")
        # from rgb-depth images, inference the results and publish it
        self.ts.registerCallback(self.rgbd_callback)
        self.idx = 1

    def rgbd_callback(self, rgb_msg, depth_msg):
        rospy.loginfo("[INFO] Get new image")
        if ord(getch.getch()) == 115:
            image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            rospy.loginfo("[INFO] capture and save {}-th image".format(self.idx))
            cv2.imwrite("/home/demo/furniture-real/rgb/{}.png".format(self.idx), image)
            image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1') # [1536, 2048]
            image_array = np.array(image, dtype = np.dtype('f8'))
            np.save("/home/demo/furniture-real/depth_value/{}.npy".format(self.idx), image_array)
            cv_image_norm = cv2.normalize(image_array, image_array, 0, 255, cv2.NORM_MINMAX)
            cv2.imwrite("/home/demo/furniture-real/depth/{}.png".format(self.idx), cv_image_norm)
            self.idx += 1
        else:
            pass


if __name__ == '__main__':

    receiver = Receiver()
    rospy.spin()