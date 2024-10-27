from ias_pykdl import KDLInterface
import numpy as np
import rospy
import sys,os

from pathlib import Path
path = Path(sys.path[0])
src_folder = path.parent.parent
urdf_folder = os.path.join(src_folder, 'hexapod','robot')


class RobotInterface:
    def __init__(self):
        
        model_file = urdf_folder + '/hexapod.urdf'
        name = 'hexapod'
        name_base_link = 'MP_BODY'


        name_end_effector1 = 'tibia_e_rf'     
        name_end_effector2 = 'tibia_e_rm'
        name_end_effector3 = 'tibia_e_rr'

        name_end_effector4 = 'tibia_e_lf'
        name_end_effector5 = 'tibia_e_lm'
        name_end_effector6 = 'tibia_e_lr'


        #numerical definition
        n_dof = 3
        joint_pos_min = np.array([-1.74,-1.74,-1.74])
        joint_pos_max = np.array([1.74,1.74,1.74])
        gravity = [+0., +0., -9.81]

        # Group0
        self.leg1_interface = KDLInterface(model_file,n_dof,name_base_link,name_end_effector1, joint_pos_min, joint_pos_max, gravity)
        self.leg2_interface = KDLInterface(model_file,n_dof,name_base_link,name_end_effector2, joint_pos_min, joint_pos_max, gravity)
        self.leg3_interface = KDLInterface(model_file,n_dof,name_base_link,name_end_effector3, joint_pos_min, joint_pos_max, gravity)

        self.leg4_interface = KDLInterface(model_file,n_dof,name_base_link,name_end_effector4, joint_pos_min, joint_pos_max, gravity)
        self.leg5_interface = KDLInterface(model_file,n_dof,name_base_link,name_end_effector5, joint_pos_min, joint_pos_max, gravity)
        self.leg6_interface = KDLInterface(model_file,n_dof,name_base_link,name_end_effector6, joint_pos_min, joint_pos_max, gravity)

        # Group0
        self.leg1_joint_names = ['j_c1_rf','j_thigh_rf', 'j_tibia_rf']
        self.leg2_joint_names = ['j_c1_rm','j_thigh_rm', 'j_tibia_rm'] 
        self.leg3_joint_names = ['j_c1_rr','j_thigh_rr', 'j_tibia_rr']

        self.leg4_joint_names = ['j_c1_lf','j_thigh_lf', 'j_tibia_lf']
        self.leg5_joint_names = ['j_c1_lm','j_thigh_lm', 'j_tibia_lm']
        self.leg6_joint_names = ['j_c1_lr','j_thigh_lr', 'j_tibia_lr']


        self.leg1 = {'id':str(1), 'interface':self.leg1_interface, 'joint_names':self.leg1_joint_names, 'rest_pos':[0.10167,  0.15628, -0.061269]}
        self.leg2 = {'id':str(2), 'interface':self.leg2_interface, 'joint_names':self.leg2_joint_names, 'rest_pos':[0.16354,  0.0, -0.061269]}
        self.leg3 = {'id':str(3), 'interface':self.leg3_interface, 'joint_names':self.leg3_joint_names, 'rest_pos':[0.10167 , -0.15628, -0.061269]}

        self.leg4 = {'id':str(4), 'interface':self.leg4_interface, 'joint_names':self.leg4_joint_names, 'rest_pos':[ -0.10167,  0.15628, -0.061269]}
        self.leg5 = {'id':str(5), 'interface':self.leg5_interface, 'joint_names':self.leg5_joint_names, 'rest_pos':[ -0.16354,  0.0, -0.061269]}
        self.leg6 = {'id':str(6), 'interface':self.leg6_interface, 'joint_names':self.leg6_joint_names, 'rest_pos':[ -0.10167, -0.15628, -0.061269]}

        # self.legs = [self.leg1,self.leg3,self.leg5,self.leg2,self.leg4,self.leg6]
        self.legs = [self.leg1,self.leg2,self.leg3,self.leg4,self.leg5,self.leg6]

if __name__ == "__main__":
    
    interface = RobotInterface()

    for leg in interface.legs:
        print(leg['joint_names'])

    