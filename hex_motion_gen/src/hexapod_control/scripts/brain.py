#!/usr/bin/env python3

import rospy
import math
import numpy as np
from gait_generator import GaitGenerator
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D
from bezier import Bezier
from std_msgs.msg import Float32MultiArray
import sys

from hexapod_walker import HexWalker
import csv
from bezier import Bezier
from data_saver import DataSaver
from gazebo_msgs.msg import ModelStates
import os,sys

import csv


pi = math.pi
tpi = math.pi*2


""" 6 cells """
CATER = tpi/3*np.ones(6)
TRI = pi*np.ones(6)
METACH = tpi/6*np.ones(6)
WAVE = np.array([tpi/3,tpi/3, pi, tpi/3,tpi/3, pi/3])
TETRA = np.array([tpi/3,tpi/3,0,tpi/3,tpi/3,2*tpi/3])
LURCH = np.array([pi,pi,0,pi,pi,0])

gait_dict = {'tri':{'theta':TRI, 'mu':0.5},
             'cater':{'theta':CATER, 'mu':0.6},    #0.6
             'metach':{'theta':METACH, 'mu':0.7},
             'wave':{'theta':WAVE, 'mu':0.83},
             'tetra':{'theta':TETRA, 'mu':0.66},
             'hsmetach':{'theta':METACH, 'mu':0.4},
             'lurch':{'theta':LURCH, 'mu':0.55}}

class HexBrain(object):
    def __init__(self, walker):
        self.beta = 0.995
        self.theta_pub = rospy.Publisher('/theta_command', Float32MultiArray, queue_size=10)
        self.theta_sub = rospy.Subscriber('/current_theta', Float32MultiArray, self.theta_cb, queue_size=10)
        self.ms_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_cb, queue_size=10)
        self.theta = np.zeros(6)
        self.kai = 1
        self.ts = 10

        self.bezier = Bezier()
        self.bezier.addPoint(0,0)
        self.bezier.addPoint(0.5,0)
        self.bezier.addPoint(0.5,1)
        self.bezier.addPoint(1,1)

        self.walker = walker #HexWalker()

        self.in_trans = False
        self.trans_finish = False

        """
        CSV files
        """

        data_folder = os.path.join(sys.path[0], 'data')

        f0 = open(data_folder + "/pos.csv", 'w')
        self.pos_writer = csv.writer(f0)

        f =  open(data_folder + "/vel.csv", 'w')
        self.vel_writer = csv.writer(f)
        
        f1 = open(data_folder + "/mu.csv", 'w')
        self.mu_writer = csv.writer(f1)


    def theta_cb(self, data):
        self.theta = np.array(data.data)

    def model_cb(self, data):
        self.model_position = [data.pose[1].position.x, data.pose[1].position.y, data.pose[1].position.z]
        self.model_vel = [data.twist[1].linear.x, data.twist[1].linear.y, data.twist[1].linear.z]

    def smooth_theta(self, start_theta, target_theta, progress):
        if progress < self.ts:
            t = progress/self.ts
            _,percent = self.bezier.getPos(t)

            smooth_theta = start_theta + (target_theta - start_theta)*percent
        else:
            smooth_theta = target_theta

        return smooth_theta
    
    def smooth_mu(self, start_mu, target_mu, progress):
        if progress < self.ts:
            t = progress/self.ts
            _,percent = self.bezier.getPos(t)
            smooth_mu = start_mu + (target_mu - start_mu)*percent
        else:
            smooth_mu = target_mu

        return smooth_mu

    
    def gait_transition(self, current_gait, target_gait, progress):
        
        self.trans_finish = False
        smooth_theta = np.zeros(6)
       
        current_theta = gait_dict[current_gait]['theta']
        current_mu = gait_dict[current_gait]['mu']
        target_theta = gait_dict[target_gait]['theta']
        target_mu = gait_dict[target_gait]['mu']

        smooth_theta = self.smooth_theta(current_theta, target_theta,progress)
        smooth_mu = self.smooth_mu(current_mu, target_mu,progress)

        msg = Float32MultiArray()
        msg.data = smooth_theta
        self.theta_pub.publish(msg)
        self.walker.mu = smooth_mu


        return smooth_theta
    
    def stable_gait(self, theta):
        theta_dict = {'tri':TRI, 'cater':CATER, 'metach':METACH, 'wave':WAVE, 'tetra':TETRA}

        res_key, res_val = min(theta_dict.items(), key=lambda x: np.linalg.norm(theta - x[1]))
        return res_key

