import numpy as np
import math
import matplotlib.pyplot as plt
import time
import random


def low_pass(x_new, y_old, dt, cutoff=20):
    alpha = dt / (dt + 1 / (2 * np.pi * cutoff))
    y_new = x_new * alpha + (1 - alpha) * y_old
    return y_new

def get_stance_avg_f(stance_g, f_list):
    total_f = 0
    logic_f = 1
    len_f = len(stance_g)
    
    # print(stance_g)
    # print(f_list)

    for leg in stance_g:
        leg_force = f_list[leg]
        total_f += leg_force
        logic_f = logic_f*f_logic(leg_force)
    
    stance_avg_f = total_f*logic_f/len_f
    return stance_avg_f

def f_logic(force):
    if force>0:
        f_logic = 1.0
    else:
        f_logic = 0

    return f_logic

class CubicTraj(object):
    def __init__(self, vz_swing, h):
        self.z_start_pos = None
        self.vz_swing = vz_swing
        self.h = h

    def stance_z_ref(self, z_start, t):
        # t from 0~1/2
        a = 4*(12*z_start-3*self.vz_swing)
        b = 24*(self.vz_swing/3-z_start)
        c = -self.vz_swing
        d = z_start

        z_ref = (1/3)*a*t**3 + (1/2)*b*t**2 + c*t + d
        v_ref = a*t**2 + b*t + c
        a_ref = 2*a*t + b

        return z_ref,v_ref,a_ref


    def fly_z_ref(self, t):
        # t from 0-1/3
        a = -4*27*self.h
        b = 45*self.h
        c = 0
        d = 0

        z_ref = a*t**3 + b*t**2
        v_ref = 3*a*t**2 + 2*b*t
        a_ref = 6*a*t + 2*b

        return z_ref,v_ref,a_ref


def get_phase_list(start,stop,length):
    dist = stop-start
    step = dist/(length-1)

    phase_list = np.zeros(length)
    phase_list[0] = start

    for i in range(length-1):
        phase_list[i+1] = phase_list[i]+step

    return phase_list



    

if __name__ == "__main__":
    
    # cubic_traj = CubicTraj(vz_swing=0.5,h=0.02)
    # z_start = -0.3
    # t_list = np.arange(0,1/3,0.001)

    # z_ref_list = []
    # v_ref_list = []
    # a_ref_list = []

    # for t in t_list:
    #     z_ref, v_ref, _ = cubic_traj.fly_z_ref(t)
    #     z_ref_list.append(z_ref)
    #     v_ref_list.append(v_ref)

    # plt.plot(t_list,z_ref_list)
    # plt.plot(t_list,v_ref_list)
    # plt.grid()

    # plt.show()
    f_list = [20,15,0,0,0,0]
    stance_g = [0,1]
    f = get_stance_avg_f(stance_g,f_list)
    print(f)