#! /usr/bin/env python

import rospy
from sim_detector import SimDetector
import cv_bridge
from sensor_msgs.msg import LaserScan, Image
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import TransformStamped, Pose
from tf.transformations import quaternion_from_euler
from std_srvs.srv import EmptyResponse, Empty
import tf2_ros
import numpy as np
from math import sin, cos


class DetectorNode:
    def __init__(self):
        self.image = None
        self.scan = None
        self.image_msg = None
        self.scan_msg = None
        self.target_tf = TransformStamped()
        self.detector = SimDetector()

        self.image_topic = rospy.get_param('camera/ImageTopic', default='/camera/rgb/image_raw')
        self.image_scan_topic = rospy.get_param('camera/Laser', default='/xtion/scan')

        self.bridge = cv_bridge.CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Service('detect cube', Empty, self.do_detection)

    def image_callback(self, msg):
        self.image_msg = msg

    def scan_callback(self, msg):
        self.scan_msg = msg

    def convert_image(self):
        self.image = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding='bgr8')

    def convert_scan(self):
        self.scan = (self.scan_msg.angle_min, self.scan_msg.angle_increment, self.scan_msg.ranges)

    def detect(self):
        if not self.detector.detect_cube(self.image):
            label = self.detector.label
            (x, y, w, h) = self.detector.boundingbox
            center_ind = x + int(0.5 * w)
            angle = self.scan[0] + (center_ind - 1) * self.scan[1]
            distance = sum(self.scan[2][center_ind-2: center_ind+2])/4

            pose = self.pole_to_pose(distance, angle)

            self.target_tf.header.stamp = rospy.Time.now()
            self.target_tf.header.frame_id = 'base_link'
            self.target_tf.child_frame_id = label
            self.target_tf.transform.translation = pose.position
            self.target_tf.transform.rotation = pose.orientation

            return True
        else:
            return False

    def pole_to_pose(self, distance, angle):
        p = Pose()
        p.position.x = distance * sin(angle)
        p.position.y = distance * cos(angle)
        p.orientation = quaternion_from_euler(0, 0, 0)
        return p

    def scan_to_points(self, scan_msg):
        angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
        ranges = np.asarray(scan_msg.ranges)
        x = np.sin(angles) * ranges
        y = np.cos(angles) * ranges
        return x, y

    def get_goal(self):
        goal = MoveBaseGoal()

    def do_detection(self, req):
        self.detect()
        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('test_detector_node')
    d = DetectorNode()
    rospy.spin()
