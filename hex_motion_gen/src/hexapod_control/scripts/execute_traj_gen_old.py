#!/usr/bin/env python3

import rospy
import numpy as np
import time
import math
import matplotlib.pyplot as plt

from brain import HexBrain
from traj_generator import TrajGenerator
from hexapod_walker import HexWalker
from data_saver import DataSaver
import sys,os


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
             'cater':{'theta':CATER, 'mu':0.6},   #
             'metach':{'theta':METACH, 'mu':0.7},
             'wave':{'theta':WAVE, 'mu':0.83},
             'tetra':{'theta':TETRA, 'mu':0.66},
             'hsmetach':{'theta':METACH, 'mu':0.4},
             'lurch':{'theta':LURCH, 'mu':0.55}}


if __name__ == '__main__':
    print(TETRA)

    hz = 100

    traj_index = 1

    rospy.init_node('gtexp2')
    walker = HexWalker()
    brain = HexBrain(walker)
    data_folder = os.path.join(sys.path[0], 'motor_data')
    ds = DataSaver(data_folder,traj_index)

    
    slp_time = 1/hz
    mu = 0.5
    mu_vec = []
    percentage = 0
    progress = 0
    theta_vec = []
    duration_vec = []
    mu_vec = []

    start_gait = 'lurch'
    target_gait = 'metach'

    brain.walker.mu = gait_dict[start_gait]['mu']

    print('Experimental Setting: ', 'start gait: ', start_gait, ' mu: ',brain.walker.mu, ' target_gait: ',target_gait,
             ' traj index: ', traj_index)


    input("Press enter to execute the experiment")

    start_time = time.time()

    while not rospy.is_shutdown():
        loop_start_time = time.time()
        duration = time.time()-start_time


        jc = brain.walker.leg_pose_from_phase(brain.walker.phase, 1/hz)
        brain.walker.hexapod.exec_joint_command(jc)

        data = [duration, jc['j_c1_lf'],jc['j_c1_lm'],jc['j_c1_lr'],
                          jc['j_c1_rf'],jc['j_c1_rm'],jc['j_c1_rr'],
                          jc['j_thigh_lf'],jc['j_thigh_lm'],jc['j_thigh_lr'],
                          jc['j_thigh_rf'],jc['j_thigh_rm'],jc['j_thigh_rr'],
                          jc['j_tibia_lf'],jc['j_tibia_lm'],jc['j_tibia_lr'],
                          jc['j_tibia_rf'],jc['j_tibia_rm'],jc['j_tibia_rr']]

        ds.dump_data(data)

        if 10<duration<20.5:
            trans_start_time = 10
            progress = duration-trans_start_time
            smooth_theta = brain.gait_transition(start_gait, target_gait, progress)


        print('duration: ',duration, ' mu: ', brain.walker.mu)

        # duration_vec.append(duration)
        # mu_vec.append(brain.walker.mu)

        brain.mu_writer.writerow([duration, brain.walker.mu])
        # brain.pos_writer.writerow([duration]+brain.model_position)
        # brain.vel_writer.writerow([duration]+brain.model_vel)

        # r.sleep()
        time.sleep(slp_time-(time.time()%slp_time))
        # print('dt: ', time.time()-loop_start_time)

    plt.figure()
    plt.plot(duration_vec, mu_vec)
    plt.show()
