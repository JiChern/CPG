#!/usr/bin/env python

from logging import disable
from math import atan, atan2, pi
import rospy
from geometry_msgs.msg import Pose2D, PoseStamped
from msgs_and_srvs.srv import NavGoal
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Float64, Empty

import std_srvs.srv
#from gazebo_msgs.msg import ModelStates
import numpy as np
import math
import time
import tf

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

class AutoNav(object):
    def __init__(self):
        self.target_pos = [-0.5,-0.5,0]
        self.target = [-0.4, -0.5]
        self.targets = {'target1':[-0.4,-0.5], 'target2':[-1.28,-0.75]}
        # self.targets = {'target1':[-0.2,-0.4], 'target2':[-0.25,-0.4]}
        self.target_angle = 0
        self.current_pos = [0,0,0]
        rospy.Subscriber("/hexapod/pose", Pose2D, callback=self.pose_cb)
        # rospy.Subscriber("/gazebo/model_states", ModelStates, callback=self.model_cb)
        self.turning_pub = rospy.Publisher("/turning", Float64, queue_size=10)
        self.walk_pub = rospy.Publisher("/walk_manager/start_walk", Empty, queue_size=10)
        self.stop_pub = rospy.Publisher("/walk_manager/stop_walk", Empty, queue_size=10)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

        self.stop_nav_sub = rospy.Subscriber("/auto_nav/stop",Empty, self.stop_nav_cb)
        self._stop_nav = False

        self.walk_msg = Empty()
        self.stop_msg = Empty()
        self.at_goal = False


        # self.navigation_srv = rospy.Service('/auto_nav/navigate', NavGoal, self.navigate_srv_cb)
        self.navigation_sub = rospy.Subscriber('/auto_nav/navigate', Pose2D, self.navigate_cb)

        self.tf_listener = tf.TransformListener()
        self.tf_pose = [0,0]

        self.active_commu = True

        # self.plan_sub = rospy.Subscriber('/move_base/TebLocalPlannerROS/global_plan', Path, self.plan_cb)
        

    def init_env(self):
        rospy.wait_for_message('/rtabmap/grid_map', OccupancyGrid, 1000)

    #         print(msg.poses[i].pose.position.x)
    #     pass

    # def model_cb(self, msg):
    #     x = msg.pose[1].position.x
    #     y = msg.pose[1].position.y
    #     x1 = -x-1.7004
    #     y1 = -y-0.8004
    #     # self.current_pos = [x1,y1,0]
    #     # print("x = ",x1, "  y = ", y1)

    #     q_x = msg.pose[1].orientation.x
    #     q_y = msg.pose[1].orientation.y
    #     q_z = msg.pose[1].orientation.z
    #     q_w = msg.pose[1].orientation.w

    #     roll_x, pitch_y, yaw_z = euler_from_quaternion(q_x,q_y,q_z,q_w)
    #     yaw_1 = yaw_z + 1.57
    #     # print("yaw = ",yaw_1)
    #     self.current_pos = [x1,y1, yaw_1]

    def get_plan(self, nav_goal_pose, active_commu):
        # print('waiting for goal topic')
        # rospy.wait_for_topic('/move_base_simple/goal', PoseStamped, 1000)
        if not active_commu:
            goal_pose = [2.39,-1.21]
        else:
            goal_pose = nav_goal_pose

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = goal_pose[0]
        goal_msg.pose.position.y = goal_pose[1]
        goal_msg.pose.position.z = 0.0

        goal_msg.pose.orientation.x = 0
        goal_msg.pose.orientation.y = 0
        goal_msg.pose.orientation.z = 0
        goal_msg.pose.orientation.w = 1
        print('Got goal topic')

        self.goal_pub.publish(goal_msg)


        msg = rospy.wait_for_message('/move_base/TebLocalPlannerROS/global_plan', Path, 1000)
        num_points = len(msg.poses)
        print('Got plan msg')
    
        b = num_points-1
        index_list = []
        while b>2:
            index_list.append(b)
            b = b-10

        global_plan = np.array([0,0])
        for i in reversed(index_list):
            point_x = msg.poses[i].pose.position.x
            point_y = msg.poses[i].pose.position.y
            point = np.array([point_x,point_y])
            global_plan = np.vstack((global_plan, point))

        global_plan = np.delete(global_plan,0,0)
        num_gl_pts = np.shape(global_plan)[0]
        print(global_plan)

        try:
            (trans,rot) = self.tf_listener.lookupTransform('/map', '/odom', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        odom_x = trans[0]
        odom_y = trans[1]

        print('odom_x=',odom_x, '   ', 'odom_y=',odom_y)

        _,_, angle = euler_from_quaternion(rot[0], rot[1], rot[2], rot[3])
        print('rotation angle', angle)
        rotation_angle = -angle
        
        rotation_matrix = np.array([[np.cos(rotation_angle),-np.sin(rotation_angle)],
                                    [np.sin(rotation_angle),np.cos(rotation_angle)]])
        print(rotation_matrix)
        # print('rotation_mat:', rotation_matrix)
        
        test_plan = np.zeros(np.shape(global_plan))


        local_plan = np.zeros(np.shape(global_plan))
        
        for i in range(num_gl_pts):
            # local_plan[i,0] = global_plan[i,0] - x
            # local_plan[i,1] = global_plan[i,1] - y
            # local_plan[i,0] = global_plan[i,1] - y
            # local_plan[i,1] = -(global_plan[i,0] - x)

            global_vector = np.array([global_plan[i,:]]).transpose()
            print('global_vector', global_vector)
            # print('gv', global_vector)
            test_plan[i,:] = np.matmul(rotation_matrix,global_vector).transpose()
            # local_plan[i,0] = -(test_plan[i,1] + y)
            # local_plan[i,1] = (test_plan[i,0] + x)
            a = math.atan(global_plan[i,0]/global_plan[i,1])
            alpha = a + angle
            l = math.sqrt(global_plan[i,1]*global_plan[i,1]+global_plan[i,0]*global_plan[i,0])
            y = l*math.cos(alpha) #+ odom_y
            x = l*math.sin(alpha) # odom_x
            # local_plan[i,0] = x + odom_x
            # local_plan[i,1] = y + odom_y
            local_plan[i,0] = -(test_plan[i,0] - odom_x)     #- is coordination trans from odom to navigation coord (180 degree rotate)
            local_plan[i,1] = -(test_plan[i,1] - odom_y)
            print(alpha)
            # print('x=',x, "   ", 'y=',y)
            # print(rotated_result)
        
        print(test_plan)


        
        return local_plan

            



    def navigate(self, current_target):
        r = rospy.Rate(25) 
        self.set_heading(current_target)

        if self.distance(current_target) <0.08:
            # print("at target, turning")
            return True
        
        return False


    def navigate_srv_cb(self, req):
        r = rospy.Rate(25)
        print('wait for map')
        self.init_env()
        print('got map')

        print('wait for plan')
        nav_goal_pose = [req.pose.x/100.0, req.pose.y/100.0]
        local_plan = self.get_plan(nav_goal_pose,self.active_commu)
        print('got plan')
        print('local_plan', local_plan)

    

        

        # current_target = self.target
        begin_time = time.time()
        publish_time = 1
        end_time = begin_time + publish_time
        while time.time() < end_time: 

            self.walk_pub.publish(self.walk_msg)
            time.sleep(0.1)

        target_index = 1
        target_num, _ = local_plan.shape
        current_target = np.array([0,0])
        print('target_num = ',target_num)
        # print("publishing finished")
        while not rospy.is_shutdown():
            if self._stop_nav:
                self._stop_nav = False
                print("Navigation Service Interrupted!")
                break


            try:
                (trans,rot) = self.tf_listener.lookupTransform('/odom', '/camera_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            
            x = trans[0]
            y = trans[1]

            # print('x = ',-x, ', y = ',-y)

            # current_target = [2.43,0.76]
            current_target = [local_plan[target_index,0],local_plan[target_index,1]]
            print('current_target: ', current_target)
            
            
            
            # navigation Move
            _,_, rot = euler_from_quaternion(rot[0], rot[1], rot[2], rot[3])

            # current_x = -x
            # current_y = -y
            current_x = -x
            current_y = -y
            current_theta = rot

            # print('x = ', current_x, ', y = ', current_y,', theta = ', current_theta)
            self.current_pos[0] = current_x
            self.current_pos[1] = current_y
            self.current_pos[2] = current_theta
            print('current_pose1:', trans)
            print('current_pose2:', self.current_pos)
            r.sleep()

            navigate_state = self.navigate(current_target)
            if target_index == target_num - 1:
                if navigate_state == True:
                    print('At Goal Stopping')
                    begin_time = time.time()
                    publish_time = 3
                    end_time = begin_time + publish_time
                    while time.time() < end_time: 
                        self.stop_pub.publish(self.stop_msg)
                        time.sleep(0.1)

                    break
            else:            
                if navigate_state == True:
                    target_index += 1

    def navigate_cb(self, msg):
        r = rospy.Rate(25)
        print('wait for map')
        self.init_env()
        print('got map')

        print('wait for plan')
        nav_goal_pose = [msg.x/100.0, msg.y/100.0]
        local_plan = self.get_plan(nav_goal_pose,self.active_commu)
        print('got plan')
        print('local_plan', local_plan)

    

        

        # current_target = self.target
        begin_time = time.time()
        publish_time = 1
        end_time = begin_time + publish_time
        while time.time() < end_time: 

            self.walk_pub.publish(self.walk_msg)
            time.sleep(0.1)

        target_index = 1
        target_num, _ = local_plan.shape
        current_target = np.array([0,0])
        print('target_num = ',target_num)
        # print("publishing finished")
        while not rospy.is_shutdown():
            if self._stop_nav:
                self._stop_nav = False
                print("Navigation Service Interrupted!")
                break


            try:
                (trans,rot) = self.tf_listener.lookupTransform('/odom', '/camera_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            
            x = trans[0]
            y = trans[1]

            # print('x = ',-x, ', y = ',-y)

            # current_target = [2.43,0.76]
            current_target = [local_plan[target_index,0],local_plan[target_index,1]]
            print('current_target: ', current_target)
            
            
            
            # navigation Move
            _,_, rot = euler_from_quaternion(rot[0], rot[1], rot[2], rot[3])

            # current_x = -x
            # current_y = -y
            current_x = -x
            current_y = -y
            current_theta = rot

            # print('x = ', current_x, ', y = ', current_y,', theta = ', current_theta)
            self.current_pos[0] = current_x
            self.current_pos[1] = current_y
            self.current_pos[2] = current_theta
            print('current_pose1:', trans)
            print('current_pose2:', self.current_pos)
            r.sleep()

            navigate_state = self.navigate(current_target)
            if target_index == target_num - 1:
                if navigate_state == True:
                    print('At Goal Stopping')
                    begin_time = time.time()
                    publish_time = 3
                    end_time = begin_time + publish_time
                    while time.time() < end_time: 
                        self.stop_pub.publish(self.stop_msg)
                        time.sleep(0.1)

                    break
            else:            
                if navigate_state == True:
                    target_index += 1


    def turning(self):
        # turning_msg = Float64()
        # turning_msg.data = 1.0
        # self.turning_pub.publish(turning_msg)

        target_theta = self.target_angle
            

        ca = self.current_pos[2]
        
        dang = -(target_theta-ca)


        if dang >= 0: turning = 1
        if dang < 0: turning = -1

        # print(turning)
        turning_msg = Float64()
        turning_msg.data = turning
        self.turning_pub.publish(turning_msg)

        if abs(self.target_angle -ca)<0.02:
            for i in range(100):
                self.stop_pub.publish(self.stop_msg)

            print('Turning Finished')
            return False

        return True

    def distance(self, target):
        dist_arr = np.array([target[0]-self.current_pos[0], target[1]-self.current_pos[1]])
        dist = np.linalg.norm(dist_arr)
        return dist

    def set_heading(self,target):
        cx = self.current_pos[0]
        cy = self.current_pos[1]
        ca = self.current_pos[2]

        tx = target[0]
        ty = target[1]



        heading = atan2(ty-cy, tx-cx)
        
        
        dang = heading - ca

        # print("cx", cx, "cy",cy, "dang = ", dang)

        while dang < -pi: dang += 2.*pi
        while dang > pi: dang -= 2.*pi

        if abs(dang) > 0.05:
            turning = 2.*dang/pi
        else:
            turning = 0.0

        # np.clip(turning, -0.8, 0.8)

        if turning > 0.9: turning = 0.9
        if turning < -0.9: turning = -0.9

        # print(turning)
        turning_msg = Float64()
        turning_msg.data = turning
        self.turning_pub.publish(turning_msg)
        pass


    def pose_cb(self, msg):
        self.current_pos[0] = msg.x
        self.current_pos[1] = msg.y
        self.current_pos[2] = msg.theta

    def stop_nav_cb(self, msg):
        self._stop_nav = True
        



if __name__ == '__main__':
    print('start sleep')
    time.sleep(3)
    print('sleep finished')
    rospy.init_node("navigation")
    
    auto_nav = AutoNav()


    r = rospy.Rate(25)
    while not rospy.is_shutdown():
        

        r.sleep()


 
    
