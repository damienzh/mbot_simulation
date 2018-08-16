#! /usr/bin/env python

import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Pose, Quaternion
from std_srvs.srv import EmptyResponse, Empty
import random


class TfNode:
    def __init__(self):
        self.frames = []
        self.frame_ids = []
        self.random_counter = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_br = tf2_ros.TransformBroadcaster()

        rospy.Service('add_random_frame', Empty, self.add_random_frame)
        rospy.Service('del_last_random_frame', Empty, self.del_random_frame)

    def add_random_frame(self, req):
        frame = TransformStamped()
        frame.header.frame_id = 'world'
        frame.child_frame_id = 'frame'+str(self.random_counter)
        self.random_counter += 1
        frame.transform.translation.x = random.random()
        frame.transform.translation.y = random.random()
        frame.transform.rotation.w = 1
        self.frames.append(frame)
        rospy.loginfo('added random transform from world to {}'.format(frame.child_frame_id))
        return EmptyResponse()

    def del_random_frame(self, req):
        if len(self.frames) > 0:
            tr = self.frames.pop()
            rospy.loginfo('deleted transform from {} to {}'.format(tr.header.frame_id, tr.child_frame_id))
        return EmptyResponse()

    def broadcast_frames(self):
        if len(self.frames) > 0:
            for f in self.frames:
                f.header.stamp = rospy.Time.now()
                self.tf_br.sendTransform(f)

    def register_frame(self, frame):
        if not frame.child_frame_id in self.frame_ids:
            self.frames.append(frame)
            self.frame_ids.append(frame.child_frame_id)
        else:
            print 'frame already existed'

    def deregister_frame(self, frame):
        if frame in self.frames:
            self.frames.remove(frame)
        else:
            print 'no such frame'


if __name__ == '__main__':
    rospy.init_node('tf_node')
    tf_node = TfNode()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        tf_node.broadcast_frames()
        rospy.sleep(rate)
    rospy.spin()
