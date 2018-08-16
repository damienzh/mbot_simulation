#! /usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty, EmptyResponse


class MoveTest:
    def __init__(self):
        self.goal = PoseStamped()
        self.move_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.test_server = rospy.Service('test_movebase', Empty, self.pub_send_goal)

    def set_goal(self, target_pose, frame_id='odom'):
        self.goal.header.frame_id = frame_id
        self.goal.pose = target_pose
        self.goal.header.stamp = rospy.Time.now()

    def pub_send_goal(self, req):
        pose = self.set_pose()
        self.set_goal(pose)
        self.move_publisher.publish(self.goal)
        rospy.loginfo('sent target goal')
        return EmptyResponse()

    def set_pose(self):
        p = Pose()
        p.position.x = 1
        p.position.y = 0
        rpy = (0, 0, 0)
        q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]
        return p


if __name__ == '__main__':
    rospy.init_node('test_move_simple')
    m = MoveTest()
    rospy.spin()
