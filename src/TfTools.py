#! /usr/bin/env python

import rospy
import tf2_ros
from tf_conversions import posemath
from geometry_msgs.msg import TransformStamped, Pose, Transform


class TfTools:
    def __init__(self):
        pass

    def forward_transform(self, trans1, trans2):
        """
        :param trans1: transformation from frame1 to frame2
        :param trans2: transformation from frame2 to frame3
        :return: transformation from frame1 to frame3
        :type : TransformStamped
        """
        trans3 = TransformStamped()
        frame1 = trans1.header.frame_id
        frame2 = trans1.child_frame_id
        frame3 = trans2.child_frame_id
        if not frame2 == trans2.header.frame_id:
            print('frame not match')
            return False
        trans3.header.frame_id = frame1
        trans3.child_frame_id = frame3
        trans3.header.stamp = max(trans1.header.stamp, trans2.header.stamp)
        p = posemath.fromMsg(self.transform_to_pose(trans1)) * posemath.fromMsg(self.transform_to_pose(trans2))
        trans3.transform = self.pose_to_transform(posemath.toMsg(p))
        return trans3

    def transform_to_pose(self, trans):
        """
        :param trans: TransformStamped
        :return: Pose
        """
        p = Pose()
        p.position = trans.transform.translation
        p.orientation = trans.transform.rotation

        return p

    def pose_to_transform(self, pose):
        """
        :param pose: Pose
        :return: Transform
        """
        t = Transform()
        t.translation = pose.position
        t.rotation = pose.orientation

        return t
