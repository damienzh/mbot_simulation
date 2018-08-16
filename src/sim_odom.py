#! /usr/bin/env python

import rospy
import tf2_ros
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped


class Sim_odom:
    def __init__(self):
        self.imu_topic = rospy.get_param('/mbot_topic/imu', default='/imu')
        self.odom_broadcaster = tf2_ros.TransformBroadcaster()
