#!/usr/bin/env python

import rospy
import math
from robot_config import *
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

    raw_input('move in x by 5cm')
    _, init, final = move_in_cartesian(_cartesian_client, dx=0.05, arm='left')
    print init
    print '================='
    print final

    raw_input('move in x by 5cm')
    _, init, final = move_in_cartesian(_cartesian_client, dx=0.05, arm='right')
    print init
    print '================='
    print final



#def main():
#    '''
#    move gripper in x, y, z direction
#    be default, x, y, z is wrt the arm_base_link
#    to convert it to wrt base_link, the following conversion is needed
#    To move in position x direction wrt base_link: assign value to positive z
#    To move in position y direction wrt base_link: assign value to negative x
#    To move in position z direction wrt base_link: assign value to negative y
#
#    '''
#    rospy.init_node('cart_test')
#
#    arm_left_pub = rospy.Publisher('/movo/left_arm/cartesian_vel_cmd', JacoCartesianVelocityCmd, queue_size=10)
#    arm_right_pub = rospy.Publisher('/movo/right_arm/cartesian_vel_cmd', JacoCartesianVelocityCmd, queue_size=10)
#
#    left_arm_cmd = JacoCartesianVelocityCmd()
#    #left_arm_cmd.header.frame_id = 'base_link'
#    right_arm_cmd = JacoCartesianVelocityCmd()
#    #right_arm_cmd.header.frame_id = 'base_link'
#
#    left_arm_cmd.y = 0.05
#    #right_arm_cmd.x = 0.02
#    right_arm_cmd.y = 0.05
#    #raw_input('go on')
#    #arm_left_pub.publish(left_arm_cmd)
#
#    for i in range(50):
#        raw_input('go on')
#        arm_left_pub.publish(left_arm_cmd)
#        #arm_right_pub.publish(right_arm_cmd)




if __name__ == "__main__":
    #main()
    main_test()
