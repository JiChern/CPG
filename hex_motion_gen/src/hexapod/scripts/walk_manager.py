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

PI = 3.14159265
TURN_TOL = 0.005
TURN_SLOPE = 0.8
SPEED_SLOPE = 0.4
MAX_SPEED = 1.5
TURN_SLEW = 0.8
# IKSOLVER 
MAXITER = 300
TOLERANCE = 0.001
SS_TIME = 2


class Walker(object):
    def __init__(self):
        self.robot = Hexapod()
        self.interface = RobotInterface()
        self.ground_group = 0
    

        self.time = 0
        self.sstime = 0
        
        self.speed = 0
        self.turning = 0
        self.smoothturning = 0
        # self.standheight = 0.0387955
        self.standheight = -self.interface.leg1['rest_pos'][2]
        self.legraise = 0.02
        self.sweepmodifier = None
        self.fdf = 1

        self.b2d_walk_up = Bezier()
        self.b2d_walk_down = Bezier()

        self.b2d_walk_up.addPoint(-0.83775,0)
        self.b2d_walk_up.addPoint(-1.11701,0)
        self.b2d_walk_up.addPoint(-1.39626,0)
        self.b2d_walk_up.addPoint(0,3.2)
        self.b2d_walk_up.addPoint(1.39626,0)
        self.b2d_walk_up.addPoint(1.11701,0)
        self.b2d_walk_up.addPoint(0.83775,0)

        self.b2d_walk_down.addPoint(0.83775,0)
        self.b2d_walk_down.addPoint(-0.83775,0)

        self.leg_traj_x = {'1':[],'3':[],'5':[],'2':[],'4':[],'6':[]}
        self.leg_traj_y = {'1':[],'3':[],'5':[],'2':[],'4':[],'6':[]}
        self.leg_traj_z = {'1':[],'3':[],'5':[],'2':[],'4':[],'6':[]}


        #ros communication
        rospy.Subscriber('/turning', Float64, callback=self.turning_cb)

        self.pose_pub = rospy.Publisher('/hexapod/pose', Pose2D, queue_size=10)


        # odom
        self.dr_ang = 0
        self.dr_x = 0
        self.dr_y = 0
        
        


        
        
    def step(self,dt):
        #clamp speed and turning
        turn_step = -(self.smoothturning - self.turning)
        if turn_step > TURN_SLEW*dt: turn_step = TURN_SLEW*dt
        if turn_step < -TURN_SLEW*dt: turn_step = -TURN_SLEW*dt
        self.smoothturning += turn_step
        
        # Turning Math
        if abs(self.smoothturning) <= TURN_TOL:
            turn_dist = math.tan((1.0-TURN_TOL)*PI/2.0)  #*50.0
        elif abs(self.smoothturning) > TURN_TOL:
            turn_dist = math.tan((1.0-self.smoothturning)*PI/2.0)   #*50.
        # print(turn_dist)

        max_dist = 0.0
        for leg in self.interface.legs:
            dist = math.sqrt(pow(leg['rest_pos'][0]-turn_dist,2)+pow(leg['rest_pos'][1],2))
            if dist > max_dist: max_dist = dist
        
        step_size = 0.014

        max_sweep = -step_size/max_dist
        if turn_dist<0.0: max_sweep = -max_sweep

        self.time += dt
        if self.time%(2*self.fdf)<self.fdf: self.ground_group = 0
        else: self.ground_group = 1 
        # print('GG:', self.ground_group)
        

        # DR calculation
        self.dr_ang += dt*max_sweep*2*0.83775/self.fdf
        if self.dr_ang > np.pi: self.dr_ang -= 2*np.pi
        if self.dr_ang < -np.pi: self.dr_ang += 2*np.pi
        print('1',self.dr_x)
        print('ms',max_sweep)
        print('td',turn_dist)
        print('back',2*math.cos(self.dr_ang)*0.83775/self.fdf)
        print('dt',dt)
        self.dr_x = self.dr_x + max_sweep*turn_dist*dt*2*math.cos(self.dr_ang)*0.83775/self.fdf
        self.dr_y = self.dr_y + max_sweep*turn_dist*dt*2*math.sin(self.dr_ang)*0.83775/self.fdf
        print('2',self.dr_x)
        # print("max_sweep", max_sweep)
        # print("turn_dist", turn_dist)
        # print("dr_ang", self.dr_ang)

        
        pose_2d = Pose2D()
        pose_2d.x = self.dr_x
        pose_2d.y = self.dr_y
        pose_2d.theta = self.dr_ang
        self.pose_pub.publish(pose_2d)


        ii=0


        desired_angle_dict = {}

        for leg in self.interface.legs:
            
            # creating tripot gait with half period phase-shift
            if ii<3:
                cycle_time = self.time%(2*self.fdf)
            else:
                cycle_time = (self.time+self.fdf)%(2*self.fdf)

            ii+=1

            if cycle_time < self.fdf: 
                tht_pos, z_pos = self.b2d_walk_down.getPos(cycle_time/self.fdf)
            else:
                tht_pos, z_pos = self.b2d_walk_up.getPos((cycle_time-self.fdf)/(self.fdf))
            
            tht_pos *= max_sweep

            dist = math.sqrt(pow(turn_dist-leg['rest_pos'][0],2)+pow(leg['rest_pos'][1],2))

            tht0 = math.atan2(turn_dist-leg['rest_pos'][0], leg['rest_pos'][1])
            
            
            x_tar = turn_dist-dist*math.sin(tht_pos+tht0)
            y_tar = dist * math.cos(tht_pos+tht0)
            z_tar = -self.standheight + self.legraise*z_pos
            
            # if leg['id'] == '1':
            #     jnt_angs = self.robot.get_angles()
            #     angs = []
            #     for jnt_name in leg['joint_names']: angs.append(jnt_angs[jnt_name])
            #     leg_pos,_,_ = leg['interface'].forward_kinematics(angs)



            # save trajactories for testing
            self.leg_traj_x[leg['id']].append(x_tar)
            self.leg_traj_y[leg['id']].append(y_tar)
            self.leg_traj_z[leg['id']].append(z_tar)


            converged, jnt_angle = self.IKSolve(leg, np.array([x_tar,y_tar,z_tar]))

            desired_angle_dict[leg['joint_names'][0]] = jnt_angle[0] 
            desired_angle_dict[leg['joint_names'][1]] = jnt_angle[1]
            desired_angle_dict[leg['joint_names'][2]] = jnt_angle[2]




            # jn_req = servo_srvRequest()

            # jn_req.id[0] = self.robot.joint_map[leg['joint_names'][0]]
            # jn_req.id[1] = self.robot.joint_map[leg['joint_names'][1]]
            # jn_req.id[2] = self.robot.joint_map[leg['joint_names'][2]]
            # jn_req.position[0] = jnt_angle[0]
            # jn_req.position[1] = jnt_angle[1]
            # jn_req.position[2] = jnt_angle[2]

            # jn_req.velocity[0] = 200
            # jn_req.velocity[1] = 200
            # jn_req.velocity[2] = 200


            # self.robot.command_client1(jn_req)

            # print(dt)
        jn_req = servo_srvRequest()

        ii = 0
        for key in desired_angle_dict:
            jn_req.id[ii] = self.robot.joint_map[key]
            jn_req.position[ii] = desired_angle_dict[key]
            jn_req.velocity[ii] = int(round(dt*1000))
            
            ii += 1

        self.robot.command_client1(jn_req)
        # print(dt)
            # self.robot.command_client1(jn_req1)
            # self.robot.command_client(jn_req2)
            # self.robot.command_client(jn_req3)
            
            # self.robot._pub_joints[leg['joint_names'][0]].publish(jnt_angle[0])
            # self.robot._pub_joints[leg['joint_names'][1]].publish(jnt_angle[1])
            # self.robot._pub_joints[leg['joint_names'][2]].publish(jnt_angle[2])


    def reset(self):
        # TODO coding the resetting function, use cartesian controllers
        jnt_angs = self.robot.get_angles()
        # print(jnt_angs)
        
        if self.ground_group == 1:
            target_angs = []
            angss = []
            for i, leg in enumerate(self.interface.legs):
                if i>2:
                    break
                angs = []
                for jnt_name in leg['joint_names']: angs.append(jnt_angs[jnt_name])
                leg_pos,_,_ = leg['interface'].forward_kinematics(angs)
                target_pos = [leg_pos[0],leg_pos[1], leg_pos[2]+0.015]
                _,target_ang = self.IKSolve(leg,target_pos)
                target_angs.append(target_ang)
                angss.append(angs)

            r = rospy.Rate(10)
            start_time = time.time()

            while not rospy.is_shutdown():  

                t = time.time()-start_time
                # print('raise group 0 ')
                if t>=0 and t<2:
                    # print('move up')

                    angt1,_ = util.bezier_curve(np.array(angss[0]),np.array(target_angs[0]),np.zeros(3),np.zeros(3),2,t)
                    angt3,_ = util.bezier_curve(np.array(angss[1]),np.array(target_angs[1]),np.zeros(3),np.zeros(3),2,t)
                    angt5,_ = util.bezier_curve(np.array(angss[2]),np.array(target_angs[2]),np.zeros(3),np.zeros(3),2,t)

                elif t>=2 and t<4:
                    # print('move down')
                    angt1,_ = util.bezier_curve(np.array(target_angs[0]),self.robot.leg1_reset_values,np.zeros(3),np.zeros(3),2,t-2)
                    angt3,_ = util.bezier_curve(np.array(target_angs[1]),self.robot.leg3_reset_values,np.zeros(3),np.zeros(3),2,t-2)
                    angt5,_ = util.bezier_curve(np.array(target_angs[2]),self.robot.leg5_reset_values,np.zeros(3),np.zeros(3),2,t-2)
               
                else:
                    break

                # self.robot._pub_joints['j_c1_lf'].publish(angt1[0])
                # self.robot._pub_joints['j_thigh_lf'].publish(angt1[1])
                # self.robot._pub_joints['j_tibia_lf'].publish(angt1[2])
                # self.robot._pub_joints['j_c1_rm'].publish(angt3[0])
                # self.robot._pub_joints['j_thigh_rm'].publish(angt3[1])
                # self.robot._pub_joints['j_tibia_rm'].publish(angt3[2])
                # self.robot._pub_joints['j_c1_lr'].publish(angt5[0])
                # self.robot._pub_joints['j_thigh_lr'].publish(angt5[1])
                # self.robot._pub_joints['j_tibia_lr'].publish(angt5[2])

                jn_req = servo_srvRequest()

                jn_req.id[0] = self.robot.joint_map['j_c1_lf']
                jn_req.id[1] = self.robot.joint_map['j_thigh_lf']
                jn_req.id[2] = self.robot.joint_map['j_tibia_lf']
                jn_req.id[3] = self.robot.joint_map['j_c1_rm']
                jn_req.id[4] = self.robot.joint_map['j_thigh_rm']
                jn_req.id[5] = self.robot.joint_map['j_tibia_rm']
                jn_req.id[6] = self.robot.joint_map['j_c1_lr']
                jn_req.id[7] = self.robot.joint_map['j_thigh_lr']
                jn_req.id[8] = self.robot.joint_map['j_tibia_lr']

                jn_req.position[0] = angt1[0]
                jn_req.position[1] = angt1[1]
                jn_req.position[2] = angt1[2]
                jn_req.position[3] = angt3[0]
                jn_req.position[4] = angt3[1]
                jn_req.position[5] = angt3[2]
                jn_req.position[6] = angt5[0]
                jn_req.position[7] = angt5[1]
                jn_req.position[8] = angt5[2]


                jn_req.velocity[0] = 50
                jn_req.velocity[1] = 50
                jn_req.velocity[2] = 50
                jn_req.velocity[3] = 50
                jn_req.velocity[4] = 50
                jn_req.velocity[5] = 50
                jn_req.velocity[6] = 50
                jn_req.velocity[7] = 50
                jn_req.velocity[8] = 50

                self.robot.command_client1(jn_req)

                r.sleep()

            target_angs = []
            angss = []
            for i, leg in enumerate(self.interface.legs):
                if i<=2:
                    continue
                angs = []
                for jnt_name in leg['joint_names']: angs.append(jnt_angs[jnt_name])
                leg_pos,_,_ = leg['interface'].forward_kinematics(angs)
                target_pos = [leg_pos[0],leg_pos[1], leg_pos[2]+0.015]
                _,target_ang = self.IKSolve(leg,target_pos)
                target_angs.append(target_ang)
                angss.append(angs)

            r = rospy.Rate(10)
            start_time = time.time()

            while not rospy.is_shutdown():  
                t = time.time()-start_time
                print('raise group 1')
                if t>=0 and t<2:
                    angt2,_ = util.bezier_curve(np.array(angss[0]),np.array(target_angs[0]),np.zeros(3),np.zeros(3),2,t)
                    angt4,_ = util.bezier_curve(np.array(angss[1]),np.array(target_angs[1]),np.zeros(3),np.zeros(3),2,t)
                    angt6,_ = util.bezier_curve(np.array(angss[2]),np.array(target_angs[2]),np.zeros(3),np.zeros(3),2,t)

                elif t>=2 and t<4:
                    angt2,_ = util.bezier_curve(np.array(target_angs[0]),self.robot.leg2_reset_values,np.zeros(3),np.zeros(3),2,t-2)
                    angt4,_ = util.bezier_curve(np.array(target_angs[1]),self.robot.leg4_reset_values,np.zeros(3),np.zeros(3),2,t-2)
                    angt6,_ = util.bezier_curve(np.array(target_angs[2]),self.robot.leg6_reset_values,np.zeros(3),np.zeros(3),2,t-2)
                else:
                    break

                # self.robot._pub_joints['j_c1_rf'].publish(angt2[0])
                # self.robot._pub_joints['j_thigh_rf'].publish(angt2[1])
                # self.robot._pub_joints['j_tibia_rf'].publish(angt2[2])
                # self.robot._pub_joints['j_c1_rr'].publish(angt4[0])
                # self.robot._pub_joints['j_thigh_rr'].publish(angt4[1])
                # self.robot._pub_joints['j_tibia_rr'].publish(angt4[2])
                # self.robot._pub_joints['j_c1_lm'].publish(angt6[0])
                # self.robot._pub_joints['j_thigh_lm'].publish(angt6[1])
                # self.robot._pub_joints['j_thigh_lm'].publish(angt6[2])
                
                jn_req.id[0] = self.robot.joint_map['j_c1_rf']
                jn_req.id[1] = self.robot.joint_map['j_thigh_rf']
                jn_req.id[2] = self.robot.joint_map['j_tibia_rf']
                jn_req.id[3] = self.robot.joint_map['j_c1_rr']
                jn_req.id[4] = self.robot.joint_map['j_thigh_rr']
                jn_req.id[5] = self.robot.joint_map['j_tibia_rr']
                jn_req.id[6] = self.robot.joint_map['j_c1_lm']
                jn_req.id[7] = self.robot.joint_map['j_thigh_lm']
                jn_req.id[8] = self.robot.joint_map['j_tibia_lm']

                jn_req.position[0] = angt2[0]
                jn_req.position[1] = angt2[1]
                jn_req.position[2] = angt2[2]
                jn_req.position[3] = angt4[0]
                jn_req.position[4] = angt4[1]
                jn_req.position[5] = angt4[2]
                jn_req.position[6] = angt6[0]
                jn_req.position[7] = angt6[1]
                jn_req.position[8] = angt6[2]


                jn_req.velocity[0] = 50
                jn_req.velocity[1] = 50
                jn_req.velocity[2] = 50
                jn_req.velocity[3] = 50
                jn_req.velocity[4] = 50
                jn_req.velocity[5] = 50
                jn_req.velocity[6] = 50
                jn_req.velocity[7] = 50
                jn_req.velocity[8] = 50

                self.robot.command_client1(jn_req)
                r.sleep()
        
        else:

            target_angs = []
            angss = []
            for i, leg in enumerate(self.interface.legs):
                if i<=2:
                    continue
                angs = []
                for jnt_name in leg['joint_names']: angs.append(jnt_angs[jnt_name])
                leg_pos,_,_ = leg['interface'].forward_kinematics(angs)
                target_pos = [leg_pos[0],leg_pos[1], leg_pos[2]+0.015]
                _,target_ang = self.IKSolve(leg,target_pos)
                target_angs.append(target_ang)
                angss.append(angs)

            r = rospy.Rate(50)
            start_time = time.time()

            while not rospy.is_shutdown():  
                t = time.time()-start_time
                print('raise group 1')
                if t>=0 and t<2:
                    angt2,_ = util.bezier_curve(np.array(angss[0]),np.array(target_angs[0]),np.zeros(3),np.zeros(3),2,t)
                    angt4,_ = util.bezier_curve(np.array(angss[1]),np.array(target_angs[1]),np.zeros(3),np.zeros(3),2,t)
                    angt6,_ = util.bezier_curve(np.array(angss[2]),np.array(target_angs[2]),np.zeros(3),np.zeros(3),2,t)

                elif t>=2 and t<4:
                    angt2,_ = util.bezier_curve(np.array(target_angs[0]),self.robot.leg2_reset_values,np.zeros(3),np.zeros(3),2,t-2)
                    angt4,_ = util.bezier_curve(np.array(target_angs[1]),self.robot.leg4_reset_values,np.zeros(3),np.zeros(3),2,t-2)
                    angt6,_ = util.bezier_curve(np.array(target_angs[2]),self.robot.leg6_reset_values,np.zeros(3),np.zeros(3),2,t-2)

                
                else:
                    break
                jn_req = servo_srvRequest()
                jn_req.id[0] = self.robot.joint_map['j_c1_rf']
                jn_req.id[1] = self.robot.joint_map['j_thigh_rf']
                jn_req.id[2] = self.robot.joint_map['j_tibia_rf']
                jn_req.id[3] = self.robot.joint_map['j_c1_rr']
                jn_req.id[4] = self.robot.joint_map['j_thigh_rr']
                jn_req.id[5] = self.robot.joint_map['j_tibia_rr']
                jn_req.id[6] = self.robot.joint_map['j_c1_lm']
                jn_req.id[7] = self.robot.joint_map['j_thigh_lm']
                jn_req.id[8] = self.robot.joint_map['j_tibia_lm']

                jn_req.position[0] = angt2[0]
                jn_req.position[1] = angt2[1]
                jn_req.position[2] = angt2[2]
                jn_req.position[3] = angt4[0]
                jn_req.position[4] = angt4[1]
                jn_req.position[5] = angt4[2]
                jn_req.position[6] = angt6[0]
                jn_req.position[7] = angt6[1]
                jn_req.position[8] = angt6[2]


                jn_req.velocity[0] = 50
                jn_req.velocity[1] = 50
                jn_req.velocity[2] = 50
                jn_req.velocity[3] = 50
                jn_req.velocity[4] = 50
                jn_req.velocity[5] = 50
                jn_req.velocity[6] = 50
                jn_req.velocity[7] = 50
                jn_req.velocity[8] = 50

                self.robot.command_client1(jn_req)

                r.sleep()
            target_angs = []
            angss = []
            for i, leg in enumerate(self.interface.legs):
                if i>2:
                    break
                angs = []
                for jnt_name in leg['joint_names']: angs.append(jnt_angs[jnt_name])
                leg_pos,_,_ = leg['interface'].forward_kinematics(angs)
                target_pos = [leg_pos[0],leg_pos[1], leg_pos[2]+0.015]
                _,target_ang = self.IKSolve(leg,target_pos)
                target_angs.append(target_ang)
                angss.append(angs)

            r = rospy.Rate(50)
            start_time = time.time()

            while not rospy.is_shutdown():  
                t = time.time()-start_time
                print('raise group 0')
                if t>=0 and t<2:
                    angt1,_ = util.bezier_curve(np.array(angss[0]),np.array(target_angs[0]),np.zeros(3),np.zeros(3),2,t)
                    angt3,_ = util.bezier_curve(np.array(angss[1]),np.array(target_angs[1]),np.zeros(3),np.zeros(3),2,t)
                    angt5,_ = util.bezier_curve(np.array(angss[2]),np.array(target_angs[2]),np.zeros(3),np.zeros(3),2,t)

                elif t>=2 and t<4:
                    angt1,_ = util.bezier_curve(np.array(target_angs[0]),self.robot.leg1_reset_values,np.zeros(3),np.zeros(3),2,t-2)
                    angt3,_ = util.bezier_curve(np.array(target_angs[1]),self.robot.leg3_reset_values,np.zeros(3),np.zeros(3),2,t-2)
                    angt5,_ = util.bezier_curve(np.array(target_angs[2]),self.robot.leg5_reset_values,np.zeros(3),np.zeros(3),2,t-2)
                else:
                    break

                jn_req.id[0] = self.robot.joint_map['j_c1_lf']
                jn_req.id[1] = self.robot.joint_map['j_thigh_lf']
                jn_req.id[2] = self.robot.joint_map['j_tibia_lf']
                jn_req.id[3] = self.robot.joint_map['j_c1_rm']
                jn_req.id[4] = self.robot.joint_map['j_thigh_rm']
                jn_req.id[5] = self.robot.joint_map['j_tibia_rm']
                jn_req.id[6] = self.robot.joint_map['j_c1_lr']
                jn_req.id[7] = self.robot.joint_map['j_thigh_lr']
                jn_req.id[8] = self.robot.joint_map['j_tibia_lr']

                jn_req.position[0] = angt1[0]
                jn_req.position[1] = angt1[1]
                jn_req.position[2] = angt1[2]
                jn_req.position[3] = angt3[0]
                jn_req.position[4] = angt3[1]
                jn_req.position[5] = angt3[2]
                jn_req.position[6] = angt5[0]
                jn_req.position[7] = angt5[1]
                jn_req.position[8] = angt5[2]


                jn_req.velocity[0] = 50
                jn_req.velocity[1] = 50
                jn_req.velocity[2] = 50
                jn_req.velocity[3] = 50
                jn_req.velocity[4] = 50
                jn_req.velocity[5] = 50
                jn_req.velocity[6] = 50
                jn_req.velocity[7] = 50
                jn_req.velocity[8] = 50

                self.robot.command_client1(jn_req)

                r.sleep()
 

    def safe_start(self):
        for i, leg in enumerate(self.interface.legs):
            if i>2:
                break
            jnt_angs = self.robot.get_angles()
            angs = []
            for jnt_name in leg['joint_names']: angs.append(jnt_angs[jnt_name])
            leg_pos, _, _ = leg['interface'].forward_kinematics(angs)
            
            target_pos1 = [leg_pos[0],leg_pos[1]+0.0055, leg_pos[2]+0.01]
            target_pos2 = [leg_pos[0],leg_pos[1]+0.011, leg_pos[2]]
            
            start_time = time.time()
            r = rospy.Rate(10)
            while not rospy.is_shutdown():
                t = time.time()-start_time
                if t<2:
                    pt,vt = util.bezier_curve(np.array(leg_pos),np.array(target_pos1),
                    np.zeros(3),np.zeros(3),2,t)
                elif t>=2 and t<4:
                    pt,vt = util.bezier_curve(np.array(target_pos1),np.array(target_pos2),
                    np.zeros(3),np.zeros(3),2,t-2)
                else:
                    break
            
                _, target_angs = self.IKSolve(leg,pt)
            
                jn_req = servo_srvRequest()

                jn_req.id[0] = self.robot.joint_map[leg['joint_names'][0]]
                jn_req.id[1] = self.robot.joint_map[leg['joint_names'][1]]
                jn_req.id[2] = self.robot.joint_map[leg['joint_names'][2]]

                jn_req.position[0] = target_angs[0]
                jn_req.position[1] = target_angs[1]
                jn_req.position[2] = target_angs[2]

                self.robot.command_client1(jn_req)
                r.sleep()


        for i, leg in enumerate(self.interface.legs):
            if i<=2:
                continue
            jnt_angs = self.robot.get_angles()
            angs = []
            for jnt_name in leg['joint_names']: angs.append(jnt_angs[jnt_name])
            leg_pos, _, _ = leg['interface'].forward_kinematics(angs)
            
            target_pos1 = [leg_pos[0],leg_pos[1]-0.0055, leg_pos[2]+0.01]
            target_pos2 = [leg_pos[0],leg_pos[1]-0.011, leg_pos[2]]
            
            start_time = time.time()
            r = rospy.Rate(10)
            while not rospy.is_shutdown():
                t = time.time()-start_time
                if t<2:
                    pt,vt = util.bezier_curve(np.array(leg_pos),np.array(target_pos1),
                    np.zeros(3),np.zeros(3),2,t)
                elif t>=2 and t<4:
                    pt,vt = util.bezier_curve(np.array(target_pos1),np.array(target_pos2),
                    np.zeros(3),np.zeros(3),2,t-2)
                else:
                    break
            
                _, target_angs = self.IKSolve(leg,pt)

                jn_req = servo_srvRequest()

                jn_req.id[0] = self.robot.joint_map[leg['joint_names'][0]]
                jn_req.id[1] = self.robot.joint_map[leg['joint_names'][1]]
                jn_req.id[2] = self.robot.joint_map[leg['joint_names'][2]]

                jn_req.position[0] = target_angs[0]
                jn_req.position[1] = target_angs[1]
                jn_req.position[2] = target_angs[2]

                self.robot.command_client1(jn_req)
                r.sleep()


            
                # self.robot._pub_joints[leg['joint_names'][0]].publish(target_angs[0])
                # self.robot._pub_joints[leg['joint_names'][1]].publish(target_angs[1])
                # self.robot._pub_joints[leg['joint_names'][2]].publish(target_angs[2])              



    def IKSolve(self, leg, target):
        converged = False
        diff = 100
        iter = 0
        joint_angles, trans, _,jacobian = self.robot.fk(leg['interface'],leg['joint_names'])

        while iter < MAXITER:
            
            error = (target-trans).reshape(3,1)
            jacobian = jacobian[0:3,:]
            cart_vel = error 
            cart_vel = cart_vel
            pseudo = np.linalg.pinv(jacobian)
            q_dot = np.matmul(pseudo, cart_vel)

            print('q_dot1',q_dot)

            q_dot = np.clip(q_dot,-0.04,0.04)

            joint_angles[0] = joint_angles[0] + q_dot[0]
            joint_angles[1] = joint_angles[1] + q_dot[1]
            joint_angles[2] = joint_angles[2] + q_dot[2]

            print('q_dot',q_dot)

            trans,_,_ = leg['interface'].forward_kinematics(joint_angles)
            jacobian = leg['interface'].jacobian(joint_angles)
            diff = math.sqrt(math.pow(target[0]-trans[0],2)+math.pow(target[1]-trans[1],2)+
                            math.pow(target[2]-trans[2],2))

            if (diff<TOLERANCE):
                converged = True
                break
            iter += 1

        return converged, joint_angles

    def turning_cb(self, msg):
        self.turning = msg.data


class WalkManager(object):
    def __init__(self):
        self.walker = Walker()
        self.options = {'RUNNING':self.running, 'STOP':self.stop, 'IDLE':self.idle,
                        'RESETTING':self.reset, 'SAFESTART':self.safe_start}
        # Initial State
        self._state = 'IDLE'
        
        self._stop = True
        self._reset = False
        self._ss = False
        self.stop_sub = rospy.Subscriber('/walk_manager/stop_walk', Empty, self.stop_walk_cb)
        self.start_sub = rospy.Subscriber('/walk_manager/start_walk', Empty, self.start_walk_cb)
        self.reset_sub = rospy.Subscriber('/walk_manager/reset', Empty, self.reset_cb)
        self.ss_sub = rospy.Subscriber('/walk_manager/safe_start', Empty, self.ss_cb)

        

    def idle(self):
        print('walker state', self._state)
        if self._ss:
            self._state = 'SAFESTART'
            self._ss = False
        
        if self._reset:
            self._state = 'RESETTING'
            self._reset = False

        pass

    def safe_start(self):
        print('walker state', self._state)
        self.walker.safe_start()
        self._state = 'STOP'
        self._stop = True
        pass

    def reset(self):
        print('walker state', self._state)
        self.walker.reset()
        self._state = 'IDLE'
        self._reset = False
        pass

    def running(self):
        print('walker state', self._state)
        self.walker.step(self.dt)
        self._state = 'RUNNING'
        if self._stop:
            self._state = 'STOP'
        
    
    def stop(self):
        print('walker state', self._state)
        print(self.walker.ground_group)
        if not self._stop:
            self._state = 'RUNNING'
        if self._reset:
            self._state = 'RESETTING'

    def state_machine(self):
        last_time = time.time()
        r = rospy.Rate(25)
        while not rospy.is_shutdown():
            current_time = time.time()
            self.dt = current_time - last_time
            if self.dt>(0.08): self.dt = 0.04
            self.options[self._state]()
            last_time = current_time
            r.sleep()


    # ROS Callbacks
    def stop_walk_cb(self, msg):
        self._stop = True
    
    def start_walk_cb(self, msg):
        self._stop = False

    def reset_cb(self, msg):
        self._reset = True

    def ss_cb(self, msg):
        self._ss = True


if __name__ == '__main__':
    rospy.init_node('walker_manager')
    wm = WalkManager()
    wm.state_machine()





    # walker.safe_start()

    # Animate moving trajs of legs 

    # print(walker.leg_traj_x['3'][i],walker.leg_traj_y['3'][i], walker.leg_traj_z['3'][i])
    # print(walker.leg_traj_x['6'][i],walker.leg_traj_y['6'][i], walker.leg_traj_z['6'][i])

    # fig = plt.figure()
    # ax = Axes3D(fig)
    
    
    
    # def update(i):
    #     ax.clear()
    #     # ax.set_xlim([-0.2,0.2])
    #     # ax.set_ylim([-0.2,0.2])
    #     # ax.set_zlim([0,0.15])
    #     ax.plot3D(walker.leg_traj_x['1'], walker.leg_traj_y['1'],walker.leg_traj_z['1'])
    #     ax.plot3D(walker.leg_traj_x['3'], walker.leg_traj_y['3'], walker.leg_traj_z['3'])
    #     ax.plot3D(walker.leg_traj_x['5'], walker.leg_traj_y['5'], walker.leg_traj_z['5'])
    #     ax.plot3D(walker.leg_traj_x['2'], walker.leg_traj_y['2'], walker.leg_traj_z['2'])
    #     ax.plot3D(walker.leg_traj_x['4'], walker.leg_traj_y['4'], walker.leg_traj_z['4'])
    #     ax.plot3D(walker.leg_traj_x['6'], walker.leg_traj_y['6'], walker.leg_traj_z['6'])

    #     ax.scatter(walker.leg_traj_x['1'][i],walker.leg_traj_y['1'][i], walker.leg_traj_z['1'][i],'o')
    #     ax.scatter(walker.leg_traj_x['3'][i],walker.leg_traj_y['3'][i], walker.leg_traj_z['3'][i],'o')
    #     ax.scatter(walker.leg_traj_x['5'][i],walker.leg_traj_y['5'][i], walker.leg_traj_z['5'][i],'o')
    #     ax.scatter(walker.leg_traj_x['2'][i],walker.leg_traj_y['2'][i], walker.leg_traj_z['2'][i],'o')
    #     ax.scatter(walker.leg_traj_x['4'][i],walker.leg_traj_y['4'][i], walker.leg_traj_z['4'][i],'o')
    #     ax.scatter(walker.leg_traj_x['6'][i],walker.leg_traj_y['6'][i], walker.leg_traj_z['6'][i],'o')

    #     # plt.plot(x_list[i],y_list[i],'o')

    # ani = animation.FuncAnimation(fig,update,frames=1000,interval=20)
    # plt.show()

 
            