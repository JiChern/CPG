#!/usr/bin/env python3

import math as m
# from turtle import color

import numpy as np
import matplotlib.pyplot as  plt
from matplotlib.animation import FuncAnimation
import math
import rospy
from std_msgs.msg import Float32MultiArray, Int16
import time
import rospy

import csv


"""
ROS node for generating synchronos signals
The state and signal can be extracted in the member variables
"""
LIFTA,LIFTB = 0, 1/6
SHIFTA, SHIFTB = 1/6, 1/3
TOUCHDOWNA, TOUCHDOWNB = 1/3, 1/2
STANCEA, STANCEB = 1/2, 1
TRANSINTERVAL = 0.05

class GaitGenerator(object):
    def __init__(self,cell_num, alpha, beta, mu, omega, gamma):
        self.cell_num = cell_num
        self.alpha = alpha
        self.beta = beta
        self.mu = mu
        self.omega = omega
        self.gamma = gamma


        self.theta = np.zeros(6)
        self.phase = np.zeros(6)

        self.gait_pub = rospy.Publisher('/gait', Float32MultiArray, queue_size=10)

        self.theta_sub = rospy.Subscriber('/theta_command', Float32MultiArray, self.theta_cb, queue_size=10)
        self.theta_pub = rospy.Publisher('/current_theta', Float32MultiArray, queue_size=10)
        

        # self.gait_switch = rospy.Subscriber('')

    def theta_cb(self, data):
        self.theta = data.data        

    def rotation_mat(self, theta):
        rotation_mat = np.array([[math.cos(theta), math.sin(theta)],
                                 [-math.sin(theta), math.cos(theta)]])
        return rotation_mat

    def rotation_mat_cc(self, theta):
        rotation_mat = np.array([[math.cos(theta), -math.sin(theta)],
                                 [math.sin(theta), math.cos(theta)]])
        return rotation_mat



    def hopf(self, x,y):
        
        r_2 = x ** 2 + y ** 2
        dx = self.alpha * (self.mu - r_2) * x - self.omega * y
        dy = self.beta * (self.mu - r_2) * y + self.omega * x  

        return np.array([dx,dy]).reshape(2,1)    


    def set_theta(self, theta):
        self.theta = theta


    def normalized_difussive_hopf_coupling(self, pos, steps):
        # Given current position of all legs and step length, Compute positions at next step
        #p os = [x1, x2, x3, x4, y1, y2, y3, y4] 

        x = np.array(pos[0])
        y = np.array(pos[1])

        dx = np.zeros(self.cell_num)
        dy = np.zeros(self.cell_num)



        for i in range(self.cell_num):
            z_i = np.array([x[i],y[i]]).reshape(2,1)

            F_zi = self.hopf(x[i],y[i]) 

            R = self.rotation_mat(self.theta[i])

            if i<self.cell_num-1:
                coeff = 6-i
                z_i_ = np.array([x[i+1],y[i+1]]).reshape(2,1)
                r_z_i_ = np.matmul(R, z_i_)
                theta_l = self.calculate_theta_l_with_sign(r_z_i_, z_i)
                R_l = self.rotation_mat(theta_l)
                # dz_i = F_zi + coeff*self.gamma*R_l@(r_z_i_-z_i)

                vel_vec = r_z_i_-z_i
                vec_length = np.linalg.norm(vel_vec) + 0.001
                omege_dir_vec = np.matmul(R_l,vel_vec)/vec_length 

                omega_vec_length = vec_length * math.cos(theta_l)
                omega_vec = omega_vec_length * omege_dir_vec
                dz_i = F_zi + coeff*self.gamma*omega_vec

                

            else:
                coeff = 6-i
                z0 = np.array([x[0],y[0]]).reshape(2,1)
                r_z_0 = np.matmul(R,z0)
                theta_l = self.calculate_theta_l_with_sign(r_z_0, z_i)
                R_l = self.rotation_mat(theta_l)
                # dz_i = F_zi + coeff*self.gamma*R_l@(r_z_0-z_i)

                vel_vec = r_z_0-z_i
                vec_length = np.linalg.norm(vel_vec) + 0.001
                omege_dir_vec = np.matmul(R_l,vel_vec)/vec_length

                omega_vec_length = vec_length * math.cos(theta_l)
                omega_vec = omega_vec_length * omege_dir_vec
                dz_i = F_zi + coeff*self.gamma*omega_vec


            # print(dz_i)

            dx[i] = dz_i[0,0]
            dy[i] = dz_i[1,0]

     
        return x + dx*steps, y + dy*steps


    def normalized_difussive_hopf_coupling_soft(self, pos, steps):
        # Given current position of all legs and step length, Compute positions at next step
        #p os = [x1, x2, x3, x4, y1, y2, y3, y4] 

        x = np.array(pos[0])
        y = np.array(pos[1])

        dx = np.zeros(self.cell_num)
        dy = np.zeros(self.cell_num)



        for i in range(self.cell_num):
            z_i = np.array([x[i],y[i]]).reshape(2,1)

            

            R = self.rotation_mat_cc(self.theta[i])

            if i<self.cell_num-1:
                coeff = i+1
                z_i_ = np.array([x[i+1],y[i+1]]).reshape(2,1)
                r_z_i = np.matmul(R, z_i)


                theta_l = self.calculate_theta_l_with_sign(z_i_, r_z_i)
                R_l = self.rotation_mat_cc(theta_l)
                # dz_i = F_zi + coeff*self.gamma*R_l@(r_z_i_-z_i)

                vel_vec = r_z_i-z_i_

                vec_length = np.linalg.norm(vel_vec) + 0.001
                omege_dir_vec = np.matmul(R_l,vel_vec)/vec_length 

                omega_vec_length = vec_length * math.cos(theta_l)
                omega_vec = omega_vec_length * omege_dir_vec

                F_zi_ = self.hopf(x[i+1],y[i+1]) 

                dz_i_ = F_zi_ + coeff*self.gamma*omega_vec

                dx[i+1] = dz_i_[0,0]
                dy[i+1] = dz_i_[1,0]
                

            else:
                coeff = i+1
                z_i_ = np.array([x[0],y[0]]).reshape(2,1)
                r_z_i = np.matmul(R,z_i)
                theta_l = self.calculate_theta_l_with_sign(z_i_, r_z_i)
                R_l = self.rotation_mat_cc(theta_l)
                # dz_i = F_zi + coeff*self.gamma*R_l@(r_z_0-z_i)

                vel_vec = r_z_i-z_i_
                vec_length = np.linalg.norm(vel_vec) + 0.001
                omege_dir_vec = np.matmul(R_l,vel_vec)/vec_length

                omega_vec_length = vec_length * math.cos(theta_l)
                omega_vec = omega_vec_length * omege_dir_vec

                F_zi_ = self.hopf(x[0],y[0]) 
                
                dz_i_ = F_zi_ + coeff*self.gamma*omega_vec
                
                
                dx[0] = dz_i_[0,0]
                dy[0] = dz_i_[1,0]

            # print(dz_i)



     
        return x + dx*steps, y + dy*steps


    def cal_phase(self,x,y):
        """
        phase: 0~0.5 swing
               0.5~1 stance
        """
        if x>0 and y>=0:
            theta = math.atan(y/x)
        
        if x<0:
            theta = math.atan(y/x)+math.pi
        
        if x>0 and y<0:
            theta = math.atan(y/x) + 2*math.pi
        
        if x==0 and y>0:
            theta = math.pi/2

        if x==0 and y<0:
            theta = -math.pi/2
        
        if x==0 and y==0:
            theta = None

        phase = theta/(2*math.pi)

        return phase


    def filter(self, last_p, sampled_p):
        beta = 0.95
        phase_now = beta*last_p + (1-beta)*sampled_p

        return phase_now


    def calculate_theta_l_with_sign(self, z1, z2):
        epsilon = 0.00001
        n_z1 = np.linalg.norm(z1)
        n_z2 = np.linalg.norm(z2)

        norm = n_z1 * n_z2        
        c_theta = np.dot(z1.ravel(),z2.ravel())/(norm+epsilon)
        theta = math.acos(c_theta)


        cross = np.cross(z1.ravel(),z2.ravel())

        if cross > 0:
            theta = -theta

        return theta/2


    def update(self, steps=0.001):
        
        x,y = self.normalized_difussive_hopf_coupling(self.pos, steps=steps)
        self.pos = [x,y]
        for i in range(6):
            self.phase[i] = self.cal_phase(x[i],y[i])
        
        return self.phase

    def update_soft(self, steps=0.001):
        
        x,y = self.normalized_difussive_hopf_coupling_soft(self.pos, steps=steps)
        self.pos = [x,y]
        for i in range(6):
            self.phase[i] = self.cal_phase(x[i],y[i])
        
        return self.phase

    def start_oscillate(self, init_steps):
        '''
        Given the initial values, let the oscillator to oscillate in the first place 
        after succesfully oscillated, return the states and values
        '''
        l1_x, l1_y = 0.01, 0
        l2_x, l2_y = 0, 0
        l3_x, l3_y = 0, 0
        l4_x, l4_y = 0, 0
        l5_x, l5_y = 0, 0
        l6_x, l6_y = 0, 0

        pos =[[l1_x, l2_x, l3_x, l4_x,l5_x,l6_x], [l1_y, l2_y, l3_y, l4_y,l5_y,l6_y]]
               
        poses = []

        poses.append(pos)
        for i in range(init_steps):
            x,y = self.normalized_difussive_hopf_coupling(pos, 0.005)
            pos = [x,y]
            pos_list = [np.ndarray.tolist(x), np.ndarray.tolist(y)]
            poses.append(pos_list)

        self.pos = pos
        return poses

    def start_oscillate_soft(self, init_steps):
        '''
        Given the initial values, let the oscillator to oscillate in the first place 
        after succesfully oscillated, return the states and values
        '''
        l1_x, l1_y = 0.01, 0
        l2_x, l2_y = 0, 0
        l3_x, l3_y = 0, 0
        l4_x, l4_y = 0, 0
        l5_x, l5_y = 0, 0
        l6_x, l6_y = 0, 0

        pos =[[l1_x, l2_x, l3_x, l4_x,l5_x,l6_x], [l1_y, l2_y, l3_y, l4_y,l5_y,l6_y]]
               
        poses = []

        poses.append(pos)
        for i in range(init_steps):
            x,y = self.normalized_difussive_hopf_coupling_soft(pos, 0.005)
            pos = [x,y]
            pos_list = [np.ndarray.tolist(x), np.ndarray.tolist(y)]
            poses.append(pos_list)

        self.pos = pos
        return poses



if __name__ == '__main__':
    rospy.init_node('test_sg')
    sg = GaitGenerator(cell_num=6, alpha=10, beta=10, mu=1, omega=2*np.pi, gamma=1)
    # sg = GaitGenerator(cell_num=6, alpha=10, beta=10, mu=1, omega=np.pi*4, gamma=1)


    pi = math.pi
    tpi = math.pi*2

    """ 6 cells """
    cater = tpi/3*np.ones(6)
    tri = pi*np.ones(6)
    metach = tpi/6*np.ones(6)
    wave = np.array([tpi/3,tpi/3, pi, tpi/3,tpi/3, pi/3])
    tetrapod = np.array([tpi/3,tpi/3,0,tpi/3,tpi/3,2*tpi/3])

    lurch = np.array([pi,pi,0,pi,pi,0])

    test_theta = tpi*np.ones(6)

    
    



    
    r = rospy.Rate(1000)

    dt = 0.005


    phase_vec = np.zeros(6)

    duration = 0
    duration_vec = []
    start_time = time.time()

    l1_x, l1_y = 1, 0
    l2_x, l2_y = -1, 0
    l3_x, l3_y = 0, 0
    l4_x, l4_y = 0, 0
    l5_x, l5_y = 0, 0
    l6_x, l6_y = 0, 0
    # l7_x, l7_y = 0, 0
    # l8_x, l8_y = 0, 0
    # l9_x, l9_y = 0, 0
    # l10_x, l10_y = 0, 0
    # l11_x, l11_y = 0, 0
    # l12_x, l12_y = 0, 0

    pos =[[l1_x, l2_x, l3_x, l4_x,l5_x,l6_x], [l1_y, l2_y, l3_y, l4_y,l5_y,l6_y]]
    # pos =[  [l1_x, l2_x, l3_x, l4_x,l5_x,l6_x,l7_x,l8_x,l9_x,l10_x,l11_x,l12_x], 
    #         [l1_y, l2_y, l3_y, l4_y,l5_y,l6_y,l7_x,l8_y,l9_y,l10_y,l11_y,l12_y]]


    # pos = np.array([[l1_x, l2_x], [l1_y, l2_y]])


    phase_vec0 = []
    phase_vec1 = []
    phase_vec2 = []
    phase_vec3 = []
    phase_vec4 = []
    phase_vec5 = []

    beta = 0.995

    sg.set_theta(lurch)

    sg.start_oscillate_soft(1000)


    while not rospy.is_shutdown():
        loop_start_time = time.time()
        duration = time.time()-start_time
        print(duration)
        # if duration > 5:
        #     target_theta = metach
        #     new_theta = beta*sg.theta + (1-beta)*target_theta
        # #     # print(new_theta)
        #     sg.theta = new_theta

        phase = sg.update_soft(steps=dt)

        msg = Float32MultiArray()
        msg.data = phase
        sg.gait_pub.publish(msg)

        msg_theta = Float32MultiArray()
        msg_theta.data = sg.theta
        sg.theta_pub.publish(msg_theta)

        r.sleep()
        dt = time.time()-loop_start_time

        # phase_vec0.append(phase[0])
        # phase_vec1.append(phase[1])
        # phase_vec2.append(phase[2])
        # phase_vec3.append(phase[3])
        # phase_vec4.append(phase[4])
        # phase_vec5.append(phase[5])

    with open('/home/nvidia/catkin_ws/src/hexapod_gait/scripts/datas/data.csv', 'w') as f:
     
    # using csv.writer method from CSV package
        write = csv.writer(f)
        
        # write.writerow(fields)
        write.writerow(phase_vec0)
        write.writerow(phase_vec1)
        write.writerow(phase_vec2)
        write.writerow(phase_vec3)
        write.writerow(phase_vec4)
        write.writerow(phase_vec5)

    # while not rospy.is_shutdown():
    #     loop_start_time = time.time()
    #     duration = time.time()-start_time

        
    #     if duration < 5:
            
    #         x,y = sg.normalized_difussive_hopf_coupling_soft(pos, dt)
    #         pos = [x,y]
    #         print(sg.theta)

    #     elif 5<=duration<15:
            
    #         target_theta = metach
    #         new_theta = beta*sg.theta + (1-beta)*target_theta
    #         # print(new_theta)
    #         sg.theta = new_theta
    #         x,y = sg.normalized_difussive_hopf_coupling_soft(pos, dt) 
    #         pos = [x,y]

    #     elif 15<=duration<=25:

    #         target_theta = metach
    #         new_theta = beta*sg.theta + (1-beta)*target_theta
    #         # print(new_theta)
    #         sg.theta = new_theta
    #         x,y = sg.normalized_difussive_hopf_coupling_soft(pos, dt) 
    #         pos = [x,y]

    #     else:
  
    #         target_theta = metach
    #         new_theta = beta*sg.theta + (1-beta)*target_theta
    #         # print(new_theta)
    #         sg.theta = new_theta
    #         x,y = sg.normalized_difussive_hopf_coupling_soft(pos, dt) 
    #         pos = [x,y]
            
        
    #     print(duration)

        

    #     print(x[0])


    #     x_vec0.append(x[0])
    #     x_vec1.append(x[1])
    #     x_vec2.append(x[2])
    #     x_vec3.append(x[3])
    #     x_vec4.append(x[4])
    #     x_vec5.append(x[5]) 


    #     # x_vec6.append(x[6])
    #     # x_vec7.append(x[7])
    #     # x_vec8.append(x[8]) 



    #     duration_vec.append(duration)
        
    #     # print(x[0])

        
    #     r.sleep()
    #     dt = time.time()-loop_start_time



    # phase_vec = np.delete(phase_vec, 0, axis=0)


    # plt.figure()
    # plt.plot(duration_vec,x_vec0)
    # plt.plot(duration_vec,x_vec1)
    # plt.plot(duration_vec,x_vec2)
    # plt.plot(duration_vec,x_vec3)
    # plt.plot(duration_vec,x_vec4)
    # plt.plot(duration_vec,x_vec5)
    # plt.show()


    # plt.figure()

    # plt.subplot(6,1,1)
    # plt.plot(duration_vec,phase_vec0)
    # plt.subplot(6,1,2)
    # plt.plot(duration_vec,phase_vec1)
    # plt.subplot(6,1,3)
    # plt.plot(duration_vec,phase_vec2)
    # plt.subplot(6,1,4)
    # plt.plot(duration_vec,phase_vec3)
    # plt.subplot(6,1,5)
    # plt.plot(duration_vec,phase_vec4)
    # plt.subplot(6,1,6)
    # plt.plot(duration_vec,phase_vec5)

    
    


    # plt.show()