#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose
from controller_manager_msgs.srv import LoadController, SwitchController
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyRequest
import time

delete_srv = rospy.ServiceProxy('/gazebo/delete_model',DeleteModel,persistent=False)

spawn_srv = rospy.ServiceProxy('/gazebo/spawn_urdf_model',SpawnModel,persistent=False)

load_controller_srv = rospy.ServiceProxy('/hexapod/controller_manager/load_controller',LoadController,persistent=True)

switch_controller_srv = rospy.ServiceProxy('/hexapod/controller_manager/switch_controller',SwitchController,persistent=False)

pause_srv = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
unpause_srv = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

controller_list = ['j_c1_lf_position_controller', 'j_c1_rf_position_controller', 'j_c1_lm_position_controller',
                   'j_c1_rm_position_controller', 'j_c1_lr_position_controller', 'j_c1_rr_position_controller',
                   'j_thigh_lf_position_controller', 'j_thigh_rf_position_controller', 'j_thigh_lm_position_controller',
                   'j_thigh_rm_position_controller', 'j_thigh_lr_position_controller', 'j_thigh_rr_position_controller', 
                   'j_tibia_lf_position_controller', 'j_tibia_rf_position_controller', 'j_tibia_lm_position_controller',
                   'j_tibia_rm_position_controller', 'j_tibia_lr_position_controller', 'j_tibia_rr_position_controller',
                   'joint_state_controller']

joint_names = ['j_c1_lf', 'j_c1_rf', 'j_c1_lm',
                'j_c1_rm', 'j_c1_lr', 'j_c1_rr',
                'j_thigh_lf', 'j_thigh_rf', 'j_thigh_lm',
                'j_thigh_rm', 'j_thigh_lr', 'j_thigh_rr', 
                'j_tibia_lf', 'j_tibia_rf', 'j_tibia_lm',
                'j_tibia_rm', 'j_tibia_lr', 'j_tibia_rr']

rospy.loginfo('Creating joint command publishers')
pub_joints = {}
for j in joint_names:
    print('joint_names: ', j)
    p = rospy.Publisher('/hexapod/'+ j + '_position_controller/command', Float64, queue_size=1)
    pub_joints[j] = p
rospy.sleep(1)

def exec_joint_command(joint_command):
    for j in joint_names:
        msg = Float64()
        msg.data = joint_command[j]
        pub_joints[j].publish(msg)



if __name__ == '__main__':
    rospy.init_node('model_repawner')

    model_name = 'hexapod'
    model_xml = rospy.get_param('/robot_description')
    robot_ns = 'hexapod'
    pose = Pose()
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0.5
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1
    ref_frame = 'world'

    e = EmptyRequest()

    pause_srv(e)

    re = delete_srv(model_name)
    if re:
        print('delete model succeed')

    re = spawn_srv(model_name, model_xml, robot_ns, pose,ref_frame)
    print(re)

    unpause_srv(e)

    time.sleep(0.2)

    for controller in controller_list:
        re = load_controller_srv(controller)
        print(re)

    
    re = switch_controller_srv(controller_list,[],1,True,0)
    print(re)

    r = rospy.Rate(50)

    while not rospy.is_shutdown():
    
        joint_command = {'j_c1_lf':0, 'j_c1_lm':0, 'j_c1_lr':0, 
                        'j_c1_rf':0, 'j_c1_rm':0, 'j_c1_rr':0,
                        'j_thigh_lf':0, 'j_thigh_lm':0, 'j_thigh_lr':0,
                        'j_thigh_rr':0, 'j_thigh_rm':0, 'j_thigh_rf':0,
                        'j_tibia_lf':0, 'j_tibia_lm':0, 'j_tibia_lr':0,
                        'j_tibia_rr':0, 'j_tibia_rm':0, 'j_tibia_rf':0}
    
        exec_joint_command(joint_command)
        r.sleep()