#!/usr/bin/env python3

import numpy as np
import rospy
import tf
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class Controller():
    def __init__(self):
       
        self.rate = rospy.Rate(50)

        #订阅local_planner的话题，获取mpc计算出来的1*50的速度矩阵
        self.local_plan_sub = rospy.Subscriber('/local_plan', Float32MultiArray, self.local_planner_cb)

        #发布速度话题cmd_vel 
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        #发布当前状态话题curr_state,1*2的矩阵
        self.curr_state_pub = rospy.Publisher('/curr_state', Float32MultiArray, queue_size=10)

        self.__timer_localization = rospy.Timer(rospy.Duration(0.01), self.get_current_state)
        self.listener = tf.TransformListener()



    # ros定时器，发布当前状态x，y，1*2的矩阵
    def get_current_state(self, event):
        try:
            # 获取从'base_link'到'world'坐标系的变换
            # trans: [x, y, z]位置
            # rot: [x, y, z, w]四元数表示的旋转
            (trans, rot) = self.listener.lookupTransform('world', 'base_link', rospy.Time(0))

            curr_state = Float32MultiArray()
            curr_state.data = [trans[0], trans[1]]
            self.curr_state_pub.publish(curr_state)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    # 订阅local_planner的话题，获取速度命令，然后发布cmd_vel话题
    #不知道为什么local_planner.py不直接发布，要来这么迂回一下
    def local_planner_cb(self, msg):
        control_cmd = Twist()
        control_cmd.linear.x = msg.data[0]
        control_cmd.linear.y = msg.data[1]
        rospy.loginfo("x: %.1f, y: %.1f" % (control_cmd.linear.x, control_cmd.linear.y))
        self.vel_pub.publish(control_cmd)


if __name__ == '__main__':
    rospy.init_node('controller')
    controller = Controller()
    rospy.spin()
