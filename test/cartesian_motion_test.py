#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from movo_msgs.msg import JacoCartesianVelocityCmd
from std_msgs.msg import String

from cartesian_server.srv import *

CARTESIAN_SERVER = 'movo_cartesian'

def move_in_cartesian(_cartesian_client, dx=0, dy=0, dz=0, arm=''):
    req = CartesianGoalRequest()
    req.arm = String(arm)
    req.dx = dx
    req.dy = dy
    req.dz = dz

    res = CartesianGoalResponse()
    try:
        rospy.wait_for_service(CARTESIAN_SERVER)
        res = _cartesian_client(req)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr('Service call failed')
        return False, None, None

    return res.executed, res.pose_init, res.pose_final

def main_test():
    rospy.init_node('cart_test')
    _cartesian_client = rospy.ServiceProxy(CARTESIAN_SERVER, CartesianGoal)
    rospy.wait_for_service(CARTESIAN_SERVER)

    raw_input('right arm move in x by 5cm')
    _, init, final = move_in_cartesian(_cartesian_client, dx=0.05, arm='right')
    print 'X: ', init.pose.position.x - final.pose.position.x
    print 'Y: ', init.pose.position.y - final.pose.position.y
    print 'Z: ', init.pose.position.z - final.pose.position.z
    print '================='

    raw_input('right arm move in x by -5cm')
    _, init, final = move_in_cartesian(_cartesian_client, dx=-0.05, arm='right')
    print 'X: ', init.pose.position.x - final.pose.position.x
    print 'Y: ', init.pose.position.y - final.pose.position.y
    print 'Z: ', init.pose.position.z - final.pose.position.z
    print '================='


    raw_input('right arm move in y by -5cm')
    _, init, final = move_in_cartesian(_cartesian_client, dy=-0.05, arm='right')
    print 'X: ', init.pose.position.x - final.pose.position.x
    print 'Y: ', init.pose.position.y - final.pose.position.y
    print 'Z: ', init.pose.position.z - final.pose.position.z
    print '================='

    raw_input('right arm move in y by 5cm')
    _, init, final = move_in_cartesian(_cartesian_client, dy=0.05, arm='right')
    print 'X: ', init.pose.position.x - final.pose.position.x
    print 'Y: ', init.pose.position.y - final.pose.position.y
    print 'Z: ', init.pose.position.z - final.pose.position.z
    print '================='

    raw_input('right arm move in z by 5cm')
    _, init, final = move_in_cartesian(_cartesian_client, dz=0.05, arm='right')
    print 'X: ', init.pose.position.x - final.pose.position.x
    print 'Y: ', init.pose.position.y - final.pose.position.y
    print 'Z: ', init.pose.position.z - final.pose.position.z
    print '================='

    raw_input('right arm move in z by -5cm')
    _, init, final = move_in_cartesian(_cartesian_client, dz=-0.05, arm='right')
    print 'X: ', init.pose.position.x - final.pose.position.x
    print 'Y: ', init.pose.position.y - final.pose.position.y
    print 'Z: ', init.pose.position.z - final.pose.position.z
    print '================='

    raw_input('left arm move in x by 5cm')
    _, init, final = move_in_cartesian(_cartesian_client, dx=0.05, arm='left')
    print 'X: ', init.pose.position.x - final.pose.position.x
    print 'Y: ', init.pose.position.y - final.pose.position.y
    print 'Z: ', init.pose.position.z - final.pose.position.z
    print '================='

    raw_input('left arm move in x by -5cm')
    _, init, final = move_in_cartesian(_cartesian_client, dx=-0.05, arm='left')
    print 'X: ', init.pose.position.x - final.pose.position.x
    print 'Y: ', init.pose.position.y - final.pose.position.y
    print 'Z: ', init.pose.position.z - final.pose.position.z
    print '================='

    raw_input('left arm move in y by 5cm')
    _, init, final = move_in_cartesian(_cartesian_client, dy=0.05, arm='left')
    print 'X: ', init.pose.position.x - final.pose.position.x
    print 'Y: ', init.pose.position.y - final.pose.position.y
    print 'Z: ', init.pose.position.z - final.pose.position.z
    print '================='

    raw_input('left arm move in y by -5cm')
    _, init, final = move_in_cartesian(_cartesian_client, dy=-0.05, arm='left')
    print 'X: ', init.pose.position.x - final.pose.position.x
    print 'Y: ', init.pose.position.y - final.pose.position.y
    print 'Z: ', init.pose.position.z - final.pose.position.z
    print '================='

    raw_input('left arm move in z by 5cm')
    _, init, final = move_in_cartesian(_cartesian_client, dz=0.05, arm='left')
    print 'X: ', init.pose.position.x - final.pose.position.x
    print 'Y: ', init.pose.position.y - final.pose.position.y
    print 'Z: ', init.pose.position.z - final.pose.position.z
    print '================='

    raw_input('left arm move in z by -5cm')
    _, init, final = move_in_cartesian(_cartesian_client, dz=-0.05, arm='left')
    print 'X: ', init.pose.position.x - final.pose.position.x
    print 'Y: ', init.pose.position.y - final.pose.position.y
    print 'Z: ', init.pose.position.z - final.pose.position.z
    print '================='


if __name__ == "__main__":
    main_test()
