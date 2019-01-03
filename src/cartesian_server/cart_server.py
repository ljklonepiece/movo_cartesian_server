#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from cartesian_server.srv import *
from math import sqrt, pow, sin, cos, atan2
from std_msgs.msg import Bool, String
from cartesian_server.srv import *
from movo_msgs.msg import JacoCartesianVelocityCmd

SERVER_NAME = 'movo_cartesian'
TOOL_SERVER_NAME = 'tool_pose'

class CartesianServer(object):

    def __init__(self):

        self.P_gain = 10
        self.D_gain = 0
        self.max_vel = 0.1
        self.pose = PoseStamped()
        #self.reset = False
        self.received = False
        self.tolerance = 0.002
        self.rate = rospy.Rate(50)
        rospy.wait_for_service(TOOL_SERVER_NAME)
        self.pose_client = rospy.ServiceProxy(TOOL_SERVER_NAME, GetToolPose)
        #self.reset_sub = rospy.Subscriber('reset_cart', Bool, self.reset_callback)
        self.server = rospy.Service(SERVER_NAME, CartesianGoal, self.service_callback)
        rospy.spin()

    #def reset_callback(self, data):
    #    self.reset = data.data
    #    if self.received:
    #        self.reset = False


    def service_callback(self, req):
        self.arm = req.arm.data
        self.vel_pub = rospy.Publisher('/movo/%s_arm/cartesian_vel_cmd'%self.arm, JacoCartesianVelocityCmd, queue_size=10)
        self.received = False
        dx = req.dx
        dy = req.dy
        dz = req.dz
        result = False
        pose_init = None
        pose_final = None
        result =True
        #print dx, dy, dz
        self.pose = self.pose_client(String(self.arm)).tool_pose
        pose_init = self.pose
        if abs(dx) > 0.01 or abs(dy) > 0.01:
            result = self.move_by_xy(dx=dx, dy=dy)
        elif abs(dz) > 0.01:
            result = self.move_by_z(dz=dz)


        self.pose = self.pose_client(String(self.arm)).tool_pose
        pose_final = self.pose

        return CartesianGoalResponse(result, pose_init, pose_final)


    def move_by_xy(self, dx=0, dy=0):
        ''' move the gripper link in xy plan in a straight line'''
        vel_msg = JacoCartesianVelocityCmd()
        vel_msg.header.frame_id = 'base_link'
        self.pose = self.pose_client(String(self.arm)).tool_pose
        goal_x = self.pose.pose.position.x + dx
        goal_y = self.pose.pose.position.y + dy
        dx_prev = dx
        dy_prev = dy

        while not rospy.is_shutdown():
            #if self.reset:
            #    self.reset = False
            #    self.received = True
            #    break

            self.pose = self.pose_client(String(self.arm)).tool_pose
            dxx = goal_x - self.pose.pose.position.x
            dyy = goal_y - self.pose.pose.position.y
            err = sqrt(pow(dxx, 2) + pow(dyy, 2))

            ## PD Control
            vel_x_PD = self.P_gain * cos(atan2(dyy, dxx)) * abs(dxx) + self.D_gain * (dxx - dx_prev)
            vel_y_PD = self.P_gain * sin(atan2(dyy, dxx)) * abs(dyy) + self.D_gain * (dyy - dy_prev)

            dx_prev = dxx
            dy_prev = dyy

            if abs(vel_x_PD) > abs(self.max_vel * cos(atan2(dyy, dxx))):
                vel_x = self.max_vel * cos(atan2(dyy, dxx))
                #print 'X: constant'
            else:
                vel_x = vel_x_PD
                #print 'X: PD'

            if abs(vel_y_PD) > abs(self.max_vel * sin(atan2(dyy, dxx))):
                vel_y = self.max_vel * sin(atan2(dyy, dxx))
                #print 'Y: constant'
            else:
                vel_y = vel_y_PD
                #print 'Y: PD'

            if err < self.tolerance:
                break
            vel_msg.z = vel_x
            vel_msg.x = - vel_y
            #print vel_msg
            self.vel_pub.publish(vel_msg)
            self.rate.sleep()

        vel_msg.z = 0
        vel_msg.x = 0
        self.vel_pub.publish(vel_msg)
        rospy.sleep(0.5)
        return True

    def move_by_z(self, dz=0):
        ''' move the gripper link in z plan in a straight line'''
        vel_msg = JacoCartesianVelocityCmd()
        vel_msg.header.frame_id = 'base_link'
        self.pose = self.pose_client(String(self.arm)).tool_pose
        goal_z = self.pose.pose.position.z + dz
        dz_prev = dz

        while not rospy.is_shutdown():
            #if self.reset:
            #    self.reset = False
            #    self.received = True
            #    break

            self.pose = self.pose_client(String(self.arm)).tool_pose
            dzz = goal_z - self.pose.pose.position.z
            vel_z_PD = self.P_gain * 0.7 * dzz + self.D_gain * (dzz - dz_prev)
            dz_prev = dzz
            if abs(dzz) < self.tolerance:
                break
            if abs(vel_z_PD) > abs(self.max_vel):
                #print 'Z: constant'
                if dzz > 0:
                    vel_z = self.max_vel
                else:
                    vel_z = - self.max_vel
            else:
                #print 'Z: PD'
                vel_z = vel_z_PD

            vel_msg.y = - vel_z
            self.vel_pub.publish(vel_msg)
            self.rate.sleep()

        vel_msg.y = 0
        self.vel_pub.publish(vel_msg)
        rospy.sleep(0.5)
        return True

if __name__=='__main__':
    rospy.init_node('cart_server')
    CartesianServer()





