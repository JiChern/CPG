#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64
import numpy as np
import numpy.linalg
import util
from robot_interface import RobotInterface


import math


from ias_pykdl import KDLInterface


class Hexapod:
    """Client ROS class for manipulating Hexapod in Gazebo"""

    def __init__(self, robot_interface):
        # self.ns = ns
        self.joints = None
        self.angles = None
        self.robot_interface = robot_interface


        self._sub_joints = rospy.Subscriber('/hexapod/joint_states', JointState, self._cb_joints, queue_size=1)
        rospy.loginfo('Waiting for joints to be populated...')
        while not rospy.is_shutdown():
            if self.joints is not None:
                break
            rospy.sleep(0.1)
            rospy.loginfo('Waiting for joints to be populated...')
        rospy.loginfo('Joints populated')

        self.pub_g1 = rospy.Publisher("/group1/joint_commands", Float32MultiArray, queue_size=10)
        self.pub_g2 = rospy.Publisher("/group2/joint_commands", Float32MultiArray, queue_size=10)
        self.pub_g3 = rospy.Publisher("/group3/joint_commands", Float32MultiArray, queue_size=10)

        # rospy.loginfo('Creating joint command publishers')
        ns = '/hexapod/'
        self._pub_joints = {}
        for j in self.joints:
            p = rospy.Publisher(ns + j + '_position_controller/command', Float64, queue_size=1)
            self._pub_joints[j] = p

        print('aaaaaaaaaaaaaaaaa')
        rospy.sleep(1)

        self._pub_cmd_vel = rospy.Publisher(ns + 'cmd_vel', Twist, queue_size=1)

    def fk(self, interface, joint_names):
        angles = self.get_angles()


        joint_angles = np.zeros(3,dtype=float)
        joint_angles[0] = angles[joint_names[0]]
        joint_angles[1] = angles[joint_names[1]]
        joint_angles[2] = angles[joint_names[2]]
        trans, _, quaternions = interface.forward_kinematics(joint_angles)

        jacobian = interface.jacobian(joint_angles)
        # jacobian = jacobian[2:5,:]
        return joint_angles, trans, quaternions, jacobian

    def exec_joint_command(self, joint_command):

        for j in self.joints:
            print('j:', j)


        # jc_g1_msg, jc_g2_msg, jc_g3_msg = Float32MultiArray(),Float32MultiArray(),Float32MultiArray()
        # jc_g1_names = list(np.concatenate([self.robot_interface.leg4["joint_names"],self.robot_interface.leg1["joint_names"]]))
        # jc_g2_names = list(np.concatenate([self.robot_interface.leg5["joint_names"],self.robot_interface.leg2["joint_names"]]))
        # jc_g3_names = list(np.concatenate([self.robot_interface.leg6["joint_names"],self.robot_interface.leg3["joint_names"]]))
        
        # jc_g1, jc_g2, jc_g3 = np.zeros(6), np.zeros(6), np.zeros(6)

        # for index, name in enumerate(jc_g1_names):
        #     jc_g1[index] = joint_command[name]

        # for index, name in enumerate(jc_g2_names):
        #     jc_g2[index] = joint_command[name]

        # for index, name in enumerate(jc_g3_names):
        #     jc_g3[index] = joint_command[name]    

        # jc_g1_msg.data, jc_g2_msg.data, jc_g3_msg.data = jc_g1, jc_g2, jc_g3

        # self.pub_g1.publish(jc_g1_msg)
        # self.pub_g2.publish(jc_g2_msg)
        # self.pub_g3.publish(jc_g3_msg)

# 0.25, -1.089

    def reset_joint(self):
        joint_command = {'j_c1_lf':1.047, 'j_c1_lm':0.0, 'j_c1_lr':-1.047, 
                         'j_c1_rf':-1.047, 'j_c1_rm':0.0, 'j_c1_rr':1.047,
                         'j_thigh_lf':0.25, 'j_thigh_lm':0.25, 'j_thigh_lr':0.25,
                         'j_thigh_rr':0.25, 'j_thigh_rm':0.25, 'j_thigh_rf':0.25,
                         'j_tibia_lf':-1.089, 'j_tibia_lm':-1.089, 'j_tibia_lr':-1.089,
                         'j_tibia_rr':-1.089, 'j_tibia_rm':-1.089, 'j_tibia_rf':-1.089}

        # joint_command['j_thigh_lf']=0.573+0.3*math.sin(time.time())
        
        self.exec_joint_command(joint_command)
        


    def _cb_joints(self, msg):
        # print(msg)
        if self.joints is None:
            self.joints = msg.name
        self.angles = msg.position

    def get_angles(self):
        if self.joints is None:
            return None
        if self.angles is None:
            return None
        return dict(zip(self.joints, self.angles))


if __name__ == '__main__':
    rospy.init_node('robot', anonymous=True)
    hex_interface = RobotInterface()
    hexapod = Hexapod(hex_interface)
    

    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        joint_angles, trans, quaternions, jacobian = hexapod.fk(hex_interface.leg3['interface'],hex_interface.leg3['joint_names'])
        # print('joint angles: ', joint_angles)
        print('trans: ', trans)
        hexapod.reset_joint()

        r.sleep()


