#! /usr/bin/env python

import rospy
import cv_bridge
from sensor_msgs.msg import Image, PointCloud2, LaserScan, Imu
import sys
import cv2
import numpy as np
import time
from std_srvs.srv import Empty, EmptyResponse


class SaveData:
    def __init__(self):

        self.rgb_topic = rospy.get_param('camera/ImageTopic', default='/camera/rgb/image_raw')
        self.depth_topic = rospy.get_param('camera/DepthTopic', default='/camera/depth/image_raw')
        self.point_topic = rospy.get_param('camera/PointCloudTopic', default='/camera/depth/points')
        self.camera_scan_topic = rospy.get_param('camera/Laser', default='/xtion/scan')
        self.laser_topic = rospy.get_param('mbot/laser', default='/mbot/laser/scan')
        self.imu_topic = rospy.get_param('mbot_topic/Imu', default='/imu')

        self.bridge = cv_bridge.CvBridge()
        self.timestamp = time.strftime("%Y%m%d-%H%M%S")

        rospy.Service('save_rgb', Empty, self.save_image)
        rospy.Service('save_camera_scan', Empty, self.save_camera_scan)
        rospy.Service('save_laser_scan', Empty, self.save_laser_scan)
        rospy.loginfo('save data server initialized')

    def save_image(self, req):
        img_msg = rospy.wait_for_message(self.rgb_topic, Image)
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        self.refresh_timestamp()
        cv2.imwrite(self.timestamp+'_image.png', img)
        return EmptyResponse()

    def save_camera_scan(self, req):
        scan_msg = rospy.wait_for_message(self.camera_scan_topic, LaserScan)
        scan = np.hstack((scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment, scan_msg.ranges))
        self.refresh_timestamp()
        np.savetxt(self.timestamp+'_cameraScan.csv', scan)
        return EmptyResponse()

    def save_laser_scan(self, req):
        scan_msg = rospy.wait_for_message(self.laser_topic, LaserScan)
        scan = np.hstack((scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment, scan_msg.ranges))
        self.refresh_timestamp()
        np.savetxt(self.timestamp+'_laserScan.csv', scan)
        return EmptyResponse()

    def refresh_timestamp(self):
        self.timestamp = time.strftime("%Y%m%d-%H%M%S")


if __name__ == '__main__':
    rospy.init_node('save_data')
    s = SaveData()
    rospy.spin()
