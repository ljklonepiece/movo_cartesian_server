#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from tf.listener import TransformListener
from geometry_msgs.msg import PoseStamped
import tf
from cartesian_server.srv import *


class ToolPose(object):
    def __init__(self):
        self.pose = PoseStamped()
        self._tf_listener = TransformListener()
        self.joint_srv = rospy.Service('tool_pose', GetToolPose, self.tool_pose_callback)

    def tool_pose_callback(self, req):
        ''' get gripper link pose with respect to base_link'''
        arm = req.arm.data
        res = GetToolPoseResponse()
        time = 0
        trans = None
        rot = None
        self.pose.header.frame_id = 'base_link'
        while not rospy.is_shutdown():
            try:
                time = self._tf_listener.getLatestCommonTime('base_link', '%s_ee_link'%arm)
                trans, rot = self._tf_listener.lookupTransform('base_link', '%s_ee_link'%arm, time)
                break
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                continue

        self.pose.pose.position.x = round(trans[0], 4)
        self.pose.pose.position.y = round(trans[1], 4)
        self.pose.pose.position.z = round(trans[2], 4)
        self.pose.pose.orientation.x = round(rot[0], 4)
        self.pose.pose.orientation.y = round(rot[1], 4)
        self.pose.pose.orientation.z = round(rot[2], 4)
        self.pose.pose.orientation.w = round(rot[3], 4)
        #self.pose_pub.publish(self.pose)
        res.tool_pose = self.pose
        return res


if __name__=='__main__':
    rospy.init_node('tool_pose')
    ToolPose()
    rospy.spin()

