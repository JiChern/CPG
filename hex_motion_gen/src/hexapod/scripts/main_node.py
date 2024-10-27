#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64, Empty
from geometry_msgs.msg import Pose2D
import numpy as np
import numpy.linalg
import util
import yaml
import os

from robot_interface import RobotInterface1, RobotInterface
from hexapod import Hexapod

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D
from matplotlib import animation
import math
from bezier import Bezier
from robot_servo.srv import JointRotate, JointRotateRequest
from robot_servo.srv import servo_srv, servo_srvRequest

from msgs_and_srvs.srv import OpenClose, OpenCloseResponse
# import util

#from ias_pykdl import KDLInterface


class MainNode(object):
    def __init__(self):
        # self.walker = Walker()
        self.options = {'SLEEPING':self.sleep, 'RUNNING':self.run}
        # Initial State
        self._state = 'SLEEPING'
        self._sleep = True

        self._run = False

        self.sleep_sub = rospy.Subscriber('/main_node/sleep', Empty, self.sleep_cb)
        self.run_sub = rospy.Subscriber('/main_node/run', Empty, self.run_cb)



        self.already_sleep = False
        self.already_run = False


        

    def sleep(self):
        print('[', time.time() ,']','Robot state', self._state)

        if self.already_sleep:
            print('Already Sleeping')
            pass
        else:
            self.led_open_client(False)
            self.servo_open_client(False)

            self.already_sleep = True
            self.already_run = False

        if self._run:
            self._state = 'RUNNING'
            self._sleep = False
        

        pass

    def run(self):
        print('[', time.time() ,']','Robot state', self._state)

        if self.already_run:
            print('Already Running')
            pass
        else:
            self.led_open_client(True)
            self.servo_open_client(True)
            self.already_run = True
            self.already_sleep = False


        if self._sleep:
            self._state = 'SLEEPING'
            self._run = False
        pass


    def state_machine(self):
        last_time = time.time()
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            # current_time = time.time()
            # self.dt = current_time - last_time
            # if self.dt>(0.08): self.dt = 0.04
            self.options[self._state]()
            # last_time = current_time
            r.sleep()


    # ROS Callbacks
    
    def sleep_cb(self, msg):
        self._sleep = True

    def run_cb(self, msg):
        self._run = True

    def led_open_client(self,x):
        rospy.wait_for_service('/power_manager/set_led')
        try:
            openclose = rospy.ServiceProxy('/power_manager/set_led', OpenClose)
            resp = openclose(x)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def servo_open_client(self,x):
        rospy.wait_for_service('/power_manager/set_servo')
        try:
            openclose = rospy.ServiceProxy('/power_manager/set_servo', OpenClose)
            resp = openclose(x)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)




if __name__ == '__main__':
    rospy.init_node('walker_manager')
    main_node = MainNode()
    main_node.state_machine()

